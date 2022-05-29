#include <assert.h>
#include <errno.h>
#include <getopt.h>
#include <limits.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <sisci_error.h>
#include <sisci_api.h>

#include "c63.h"
#include "common.h"
#include "me.h"
#include "tables.h"

static uint32_t remote_node = 0;

/* getopt */
extern int optind;
extern char *optarg;

static void print_help()
{
  printf("Usage: ./c63server -r nodeid\n");
  printf("Commandline options:\n");
  printf("  -r Node id of client\n");
  printf("\n");

  exit(EXIT_FAILURE);
}

static void c63_encode_image(struct c63_common *cm, yuv_t *image)
{
  //Advance to next frame 
  destroy_frame(cm->refframe);
  cm->refframe = cm->curframe;
  cm->curframe = create_frame(cm, image);

  //Check if keyframe
  if (cm->framenum == 0 || cm->frames_since_keyframe == cm->keyframe_interval)
  {
    cm->curframe->keyframe = 1;
    cm->frames_since_keyframe = 0;

    fprintf(stderr, " (keyframe) ");
  }
  else { cm->curframe->keyframe = 0; }

  if (!cm->curframe->keyframe)
  {
    //Motion Estimation
    c63_motion_estimate(cm);
  //Motion Compensation 
    c63_motion_compensate(cm);
  }

  /* DCT and Quantization */
  //Y
  dct_quantize(image->Y, cm->curframe->predicted->Y, cm->padw[Y_COMPONENT],cm->padh[Y_COMPONENT], cm->curframe->residuals->Ydct,cm->quanttbl[Y_COMPONENT]);
  //U
  dct_quantize(image->U, cm->curframe->predicted->U, cm->padw[U_COMPONENT],cm->padh[U_COMPONENT], cm->curframe->residuals->Udct,cm->quanttbl[U_COMPONENT]);
  //V
  dct_quantize(image->V, cm->curframe->predicted->V, cm->padw[V_COMPONENT],cm->padh[V_COMPONENT], cm->curframe->residuals->Vdct,cm->quanttbl[V_COMPONENT]);

  /* Reconstruct frame for inter-prediction */
  dequantize_idct(cm->curframe->residuals->Ydct, cm->curframe->predicted->Y,cm->ypw, cm->yph, cm->curframe->recons->Y, cm->quanttbl[Y_COMPONENT]); //Y
  dequantize_idct(cm->curframe->residuals->Udct, cm->curframe->predicted->U,cm->upw, cm->uph, cm->curframe->recons->U, cm->quanttbl[U_COMPONENT]);  //U
  dequantize_idct(cm->curframe->residuals->Vdct, cm->curframe->predicted->V,cm->vpw, cm->vph, cm->curframe->recons->V, cm->quanttbl[V_COMPONENT]); //V
}
//function for freeing memory 
void free_image_data( yuv_t *image)
{
  free(image->Y);
  free(image->U);
  free(image->V);
  free(image);
}

struct c63_common* init_c63_enc(int width, int height)
{
  int i;

  /* calloc() sets allocated memory to zero */
  struct c63_common *cm = calloc(1, sizeof(struct c63_common));

  cm->width = width;
  cm->height = height;

  cm->padw[Y_COMPONENT] = cm->ypw = (uint32_t)(ceil(width/16.0f)*16); //Ypw
  cm->padh[Y_COMPONENT] = cm->yph = (uint32_t)(ceil(height/16.0f)*16); //Yph
  cm->padw[U_COMPONENT] = cm->upw = (uint32_t)(ceil(width*UX/(YX*8.0f))*8); //Upw
  cm->padh[U_COMPONENT] = cm->uph = (uint32_t)(ceil(height*UY/(YY*8.0f))*8); //Uph
  cm->padw[V_COMPONENT] = cm->vpw = (uint32_t)(ceil(width*VX/(YX*8.0f))*8);  //Vpw
  cm->padh[V_COMPONENT] = cm->vph = (uint32_t)(ceil(height*VY/(YY*8.0f))*8);  //Vph

  cm->mb_cols = cm->ypw / 8;
  cm->mb_rows = cm->yph / 8;

  /* Quality parameters -- Home exam deliveries should have original values,
   i.e., quantization factor should be 25, search range should be 16, and the
   keyframe interval should be 100. */
  cm->qp = 25;                  // Constant quantization factor. Range: [1..50]
  cm->me_search_range = 16;     // Pixels in every direction
  cm->keyframe_interval = 100;  // Distance between keyframes

  //quantization tables
  for (i = 0; i < 64; ++i)
  {
    cm->quanttbl[Y_COMPONENT][i] = yquanttbl_def[i] / (cm->qp / 10.0);
    cm->quanttbl[U_COMPONENT][i] = uvquanttbl_def[i] / (cm->qp / 10.0);
    cm->quanttbl[V_COMPONENT][i] = uvquanttbl_def[i] / (cm->qp / 10.0);
  }

  return cm;
}

int main(int argc, char **argv)
{
  int c;
  
  //SISCI declarations
  sci_desc_t v_dev;  
  sci_error_t error;
  unsigned int local_adapter_num= 0;
  size_t localOffset = 0;
  size_t remoteOffset = 0;
  /* DMA queue for image */
  sci_dma_queue_t	dmaq;         
  /* maximum entries inside dmaq */  
  unsigned int max_entries = 1; 

  /* segments for the y, u, v transfer from x86 */
  sci_remote_segment_t remote_segment;
  sci_local_segment_t local_segment;
  sci_map_t local_map;
  
  /*Communication variables */
  sci_local_segment_t local_segment_com;
  sci_remote_segment_t remote_segment_com;
  sci_map_t local_map_com;
  sci_map_t remote_map_com;
  
  /* packets for communication, defined in common.h */ 
  volatile struct com_packets *local_packets;
  volatile struct com_packets *remote_packets;
  
  /* segments for sending encoded image */
  sci_local_segment_t result_local_segment;
  sci_remote_segment_t result_remote_segment;
  sci_map_t result_local_map;
  
  if (argc == 1) { print_help(); }

  while ((c = getopt(argc, argv, "h:w:o:f:i:r:")) != -1) //extracting options
  {
    switch (c)
    {
      case 'r':
        remote_node = atoi(optarg);
        break;
      default:
        print_help(); //help in commands
        break;
    }
  }

  /* Initialize the SISCI library */
  SCIInitialize(0, &error);
  if (error != SCI_ERR_OK) {
      fprintf(stderr,"SCIInitialize failed: %s\n", SCIGetErrorString(error));
      exit(EXIT_FAILURE);
  }
  //create segment for PIO
  SCICreateSegment(v_dev,
                   &local_segment_com,
                   SEGMENT_REMOTE_COM,
                   sizeof(struct com_packets),
                   NO_CALLBACK,
                   NULL,
                   NO_FLAGS,
                   &error);
  if(error != SCI_ERR_OK){
   fprintf( stderr, "SCICreateSegment failed: %s - Error code: (0x%x)\n",
            SCIGetErrorString(error), error);
   exit(EXIT_FAILURE);
  }
  //preapare segment for PIO
  SCIPrepareSegment(local_segment_com, local_adapter_num, NO_FLAGS, &error);
  if(error != SCI_ERR_OK){
   fprintf( stderr, "SCIPrepareSegment failed: %s - Error code: (0x%x)\n",
            SCIGetErrorString(error), error);
   exit(EXIT_FAILURE);
  }

  SCISetSegmentAvailable(local_segment_com, local_adapter_num, NO_FLAGS, &error);
  if(error != SCI_ERR_OK){
   fprintf( stderr, "SCISetSegmentAvailable failed: %s - Error code: (0x%x)\n",
            SCIGetErrorString(error), error);
   exit(EXIT_FAILURE);
  }

  //Connecting to remote segment
   do {
       SCIConnectSegment(v_dev,
                         &remote_segment_com,
                         remote_node,
                         SEGMENT_LOCAL_COM,
                         local_adapter_num,
                         NO_CALLBACK,
                         NULL,
                         SCI_INFINITE_TIMEOUT,        // dont time out
                         NO_FLAGS,
                         &error);
  } while (error != SCI_ERR_OK);                     //

  //Mapping local segment
  local_packets =  SCIMapLocalSegment(local_segment_com,
                                   &local_map_com,
                                   localOffset,
                                   sizeof(struct com_packets),
                                   NULL,
                                   NO_FLAGS,
                                   &error);
  if(error != SCI_ERR_OK){
    fprintf(stderr, " failed: %s - Error code: (0x%x)\n",
            SCIGetErrorString(error), error);
    exit(EXIT_FAILURE);
  }
  remote_packets = SCIMapRemoteSegment(remote_segment_com,
                                     &remote_map_com,
                                     remoteOffset,
                                     sizeof(struct com_packets),
                                     NULL,
                                     NO_FLAGS,
                                     &error);
  if(error != SCI_ERR_OK){
    fprintf(stderr, " failed: %s - Error code: (0x%x)\n",
            SCIGetErrorString(error), error);
    exit(EXIT_FAILURE);
  }

   // Waiting til x86 have read image data, after that the data can be received from remote packets
   while(remote_packets->packet.cmd == CMD_INVALID);

   // Creating cm struct with image width and image height from x86
   struct c63_common *cm = init_c63_enc(remote_packets->packet.img_width,remote_packets->packet.img_height);

  //image segment for transfering image data to tegra through DMA
  volatile struct img_segment
  {
    uint8_t *Y[cm->padw[Y_COMPONENT]*cm->padh[Y_COMPONENT]];
    uint8_t *U[cm->padw[U_COMPONENT]*cm->padh[U_COMPONENT]];
    uint8_t *V[cm->padw[V_COMPONENT]*cm->padh[V_COMPONENT]];
  } *local_img_seg;

  
  //Resulting image segment for transfering encoded results back to x86
  
  volatile struct result_img_segment
  {
    int keyframe;
    // macroblock
    struct macroblock *mbs[COLOR_COMPONENTS][cm->mb_rows * cm->mb_cols];
    // residuals
    //Ydct
    int16_t *Ydct[cm->ypw * cm->yph];
    //Udct
    int16_t *Udct[cm->upw * cm->uph];
    //Vdct
    int16_t *Vdct[cm->vpw * cm->vph];
  } *result_local_img_seg;


  //create segment 
  SCICreateSegment(v_dev,
                   &local_segment,
                   SEGMENT_REMOTE,
                   sizeof(struct img_segment),
                   NO_CALLBACK,
                   NULL,
                   NO_FLAGS,
                   &error);
  if(error != SCI_ERR_OK){
   fprintf(stderr, "SCICreateSegment failed: %s - Error code: (0x%x)\n",
           SCIGetErrorString(error), error);
   exit(EXIT_FAILURE);
  }

  //prepare segment
  SCIPrepareSegment(local_segment, local_adapter_num, NO_FLAGS, &error);
  if(error != SCI_ERR_OK){
   fprintf(stderr, "SCIPrepareSegment failed: %s - Error code: (0x%x)\n",
           SCIGetErrorString(error), error);
   exit(EXIT_FAILURE);
  }
  
  //set 
  SCISetSegmentAvailable(local_segment, local_adapter_num, NO_FLAGS, &error);
  if(error != SCI_ERR_OK){
   fprintf(stderr, "SCISetSegmentAvailable failed: %s - Error code: (0x%x)\n",
           SCIGetErrorString(error), error);
   exit(EXIT_FAILURE);
  }

  //Create segment for image results
  SCICreateSegment(v_dev,
                  &result_local_segment,
                  SEGMENT_REMOTE_RESULT,
                  sizeof(struct result_img_segment),
                  NO_CALLBACK,
                  NULL,
                  NO_FLAGS,
                  &error);
  if(error != SCI_ERR_OK){
    fprintf(stderr, "SCICreateSegment failed: %s - Error code: (0x%x)\n",
            SCIGetErrorString(error), error);
    exit(EXIT_FAILURE);
  }
  //prepare
  SCIPrepareSegment(result_local_segment, local_adapter_num, NO_FLAGS, &error);
  if(error != SCI_ERR_OK){
    fprintf(stderr, "SCIPrepareSegment failed: %s - Error code: (0x%x)\n",
            SCIGetErrorString(error), error);
    exit(EXIT_FAILURE);
  }

  //set
  SCISetSegmentAvailable(result_local_segment,local_adapter_num,NO_FLAGS,&error);
  if(error != SCI_ERR_OK){
    fprintf(stderr, "SCISetSegmentAvailable failed: %s -Error code: (0x%x)\n",
            SCIGetErrorString(error), error);
    exit(EXIT_FAILURE);
  }

  //Mapping local segment for image data transfer from x86
  local_img_seg =  SCIMapLocalSegment(local_segment,
                                  &local_map,
                                  localOffset,
                                  sizeof(struct img_segment),
                                  NULL,
                                  NO_FLAGS,
                                  &error);
  if(error != SCI_ERR_OK){
    fprintf(stderr, "SCIMapLocalSegment failed: %s - Error code: (0x%x)\n",
            SCIGetErrorString(error), error);
    exit(EXIT_FAILURE);
  }

  //Connecting remote segment to transfer encoded image results to x86 through DMA
  do {
      SCIConnectSegment(v_dev,
                        &result_remote_segment,
                        remote_node,
                        SEGMENT_LOCAL_RESULT,
                        local_adapter_num,
                        NO_CALLBACK,
                        NULL,
                        SCI_INFINITE_TIMEOUT,
                        NO_FLAGS,
                        &error);
  } while (error != SCI_ERR_OK);

  result_local_img_seg = SCIMapLocalSegment(result_local_segment,
                                        &result_local_map,
                                        0,
                                        sizeof(struct result_img_segment),
                                        NULL,
                                        NO_FLAGS,
                                        &error);


  //DMA queue for transfering encoded image results
  
  SCICreateDMAQueue(v_dev,
                    &dmaq,
                    local_adapter_num,
                    max_entries,
                    NO_FLAGS,
                    &error
                  );
  if(error != SCI_ERR_OK){
    fprintf(stderr, "SCICreateDMAQueue failed: %s - Error code: (0x%x)\n",
            SCIGetErrorString(error), error);
    exit(EXIT_FAILURE);
  }

  // Creating image variables to use while encoding
  yuv_t *image;
  image = malloc(sizeof(*image));
  //Y
  image->Y = calloc(1, cm->padw[Y_COMPONENT]*cm->padh[Y_COMPONENT]);
  //U
  image->U = calloc(1, cm->padw[U_COMPONENT]*cm->padh[U_COMPONENT]);
  //V
  image->V = calloc(1, cm->padw[V_COMPONENT]*cm->padh[V_COMPONENT]);

  //encoding loop
  while(1)
  {
    // wait for x86 to read and transfer image data packets to Tegra
    while(local_packets->packet.cmd == CMD_INVALID);

    // Exit when x86 sends CMD_QUIT
    if(local_packets->packet.cmd == CMD_QUIT){
      break;
    }
    
    // set CMD_INVALID to tell x86 to wait
    local_packets->packet.cmd = CMD_INVALID;

   //Copying memory blocks from client segment image
   //Y
    memcpy( image->Y,local_img_seg->Y,cm->padw[Y_COMPONENT]*cm->padh[Y_COMPONENT]);
    //U
    memcpy( image->U,local_img_seg->U,cm->padw[U_COMPONENT]*cm->padh[U_COMPONENT]);
    //V
    memcpy( image->V,local_img_seg->V,cm->padw[V_COMPONENT]*cm->padh[V_COMPONENT]);

    // Encode frame
    c63_encode_image(cm, image);

    //Copying encoded result images to local result-segment
    result_local_img_seg->keyframe = cm->curframe->keyframe;

    //Copying macroblocks
    //Y
    memcpy( result_local_img_seg->mbs[Y_COMPONENT],cm->curframe->mbs[Y_COMPONENT],cm->mb_rows * cm->mb_cols * sizeof(struct macroblock));
    //U
    memcpy( result_local_img_seg->mbs[U_COMPONENT],cm->curframe->mbs[U_COMPONENT],cm->mb_rows/2 * cm->mb_cols/2 * sizeof(struct macroblock));
    //V
    memcpy( result_local_img_seg->mbs[V_COMPONENT],cm->curframe->mbs[V_COMPONENT],cm->mb_rows/2 * cm->mb_cols/2 * sizeof(struct macroblock));

    //Copying residuals
    //Ydct
    memcpy(result_local_img_seg->Ydct,cm->curframe->residuals->Ydct, cm->ypw * cm->yph * sizeof(int16_t));
    //Udct
    memcpy(result_local_img_seg->Udct,cm->curframe->residuals->Udct, cm->upw * cm->uph * sizeof(int16_t));
    //Vdct
    memcpy(result_local_img_seg->Vdct,cm->curframe->residuals->Vdct, cm->vpw * cm->vph * sizeof(int16_t));


    //Startng transfer of encoded image results from local result segment to remote result segment through DMA
    SCIStartDmaTransfer(dmaq,
                        result_local_segment,
                        result_remote_segment,
                        localOffset,
                        sizeof(struct result_img_segment),
                        remoteOffset,
                        NO_CALLBACK,
                        NULL,
                        NO_FLAGS,
                        &error);
    if(error != SCI_ERR_OK){
      fprintf(stderr,"SCIStartDmaTransfer failed: %s - Error code: (0x%x)\n",
              SCIGetErrorString(error), error);
      exit(EXIT_FAILURE);
    }

    // Waiting for DMA to finish
    SCIWaitForDMAQueue(dmaq,
                      SCI_INFINITE_TIMEOUT,
                      NO_FLAGS,
                      &error);
    if(error != SCI_ERR_OK){
      fprintf(stderr,"SCIWaitForDMAQueue failed: %s - Error code: (0x%x)\n",
              SCIGetErrorString(error), error);
      exit(EXIT_FAILURE);
    }

    // frame increments from old encode function
    ++cm->framenum;
    ++cm->frames_since_keyframe;
    
    // Telling x86 to write and read next frame
    remote_packets->packet.cmd = CMD_DONE;
  }

  //freeing memory
  free_image_data(image);

  //terminate SISCI 
  SCITerminate();


}

