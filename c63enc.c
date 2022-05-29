#include <assert.h>
#include <errno.h>
#include <getopt.h>
#include <limits.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include<time.h>
#include <sisci_error.h>
#include <sisci_api.h>
#include "c63.h"
#include "c63_write.h"
#include "common.h"
#include "tables.h"

static char *output_file, *input_file;
FILE *outfile;

static int limit_numframes = 0;

static uint32_t width;
static uint32_t height;
static uint32_t remote_node = 0;

//time measurement
double elapsed;
struct timespec start_time;
struct timespec end_time;

/* getopt */
extern int optind;
extern char *optarg;

/* Read planar YUV frames with 4:2:0 chroma sub-sampling */
static yuv_t* read_yuv(FILE *file, struct c63_common *cm)
{
  size_t len = 0;
  yuv_t *image = malloc(sizeof(*image));

  /* Read Y. The size of Y is the same as the size of the image. The indices
     represents the color component (0 is Y, 1 is U, and 2 is V) */
  image->Y = calloc(1, cm->padw[Y_COMPONENT]*cm->padh[Y_COMPONENT]);
  len += fread(image->Y, 1, width*height, file);

  /* Read U. Given 4:2:0 chroma sub-sampling, the size is 1/4 of Y
     because (height/2)*(width/2) = (height*width)/4. */
  image->U = calloc(1, cm->padw[U_COMPONENT]*cm->padh[U_COMPONENT]);
  len += fread(image->U, 1, (width*height)/4, file);

  /* Read V. Given 4:2:0 chroma sub-sampling, the size is 1/4 of Y. */
  image->V = calloc(1, cm->padw[V_COMPONENT]*cm->padh[V_COMPONENT]);
  len += fread(image->V, 1, (width*height)/4, file);

  if (ferror(file))
  {
    perror("ferror");
    exit(EXIT_FAILURE);
  }

  if (feof(file))
  {
    free(image->Y);
    free(image->U);
    free(image->V);
    free(image);

    return NULL;
  }
  else if (len != width*height*1.5)
  {
    fprintf(stderr, "Reached end of file, but incorrect bytes read.\n");
    fprintf(stderr, "Wrong input? (height: %d width: %d)\n", height, width);

    free(image->Y);
    free(image->U);
    free(image->V);
    free(image);

    return NULL;
  }

  return image;
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
//replaced the encoding part


static void print_help()
{
  printf("Usage: ./c63enc [options] input_file\n");
  printf("Commandline options:\n");
  printf("  -h                             Height of images to compress\n");
  printf("  -w                             Width of images to compress\n");
  printf("  -o                             Output file (.c63)\n");
  printf("  -r                             Node id of server\n");
  printf("  [-f]                           Limit number of frames to encode\n");
  printf("\n");

  exit(EXIT_FAILURE);
}

int main(int argc, char **argv)
{
  int c;
  yuv_t *image;
  
  /* SISCI declarations */
  sci_desc_t v_dev;  
  sci_error_t error;
  unsigned int local_adapter_num= 0;
  size_t localOffset = 0;
  size_t remoteOffset = 0;
  
  /* DMA queue for image */
  sci_dma_queue_t	dmaq;         
  
  /* maximum entries inside dmaq */  
  unsigned int maxEntries = 1; 

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
  
  /* segments for sending encoded image from tegra to x86*/
  sci_local_segment_t result_local_segment;
  sci_remote_segment_t result_remote_segment;
  sci_map_t result_local_map;


  if (argc == 1) { print_help(); }

  while ((c = getopt(argc, argv, "h:w:o:f:i:r:")) != -1)
  {
    switch (c)
    {
      case 'h':
        height = atoi(optarg);
        break;
      case 'w':
        width = atoi(optarg);
        break;
      case 'o':
        output_file = optarg;
        break;
      case 'f':
        limit_numframes = atoi(optarg);
        break;
      case 'r':
        remote_node = atoi(optarg);
        break;
      default:
        print_help();
        break;
    }
  }

  if (optind >= argc)
  {
    fprintf(stderr, "Error getting program options, try --help.\n");
    exit(EXIT_FAILURE);
  }

  outfile = fopen(output_file, "wb");

  if (outfile == NULL)
  {
    perror("fopen output file");
    exit(EXIT_FAILURE);
  }

  struct c63_common *cm = init_c63_enc(width, height);
  cm->e_ctx.fp = outfile;

  input_file = argv[optind];

  if (limit_numframes) { printf("Limited to %d frames.\n", limit_numframes); }

  FILE *infile = fopen(input_file, "rb");

  if (infile == NULL)
  {
    perror("fopen input file");
    exit(EXIT_FAILURE);
  }

  /* Initialize SISCI */
  SCIInitialize(0, &error);
  if (error != SCI_ERR_OK) {
      fprintf(stderr,"SCIInitialize failed: %s\n", SCIGetErrorString(error));
      exit(EXIT_FAILURE);
  }

  // declaring a struct of image segment to transfer image data from x86 to tegra

  volatile struct img_segment
  {
    uint8_t *Y[cm->padw[Y_COMPONENT]*cm->padh[Y_COMPONENT]];
    uint8_t *U[cm->padw[U_COMPONENT]*cm->padh[U_COMPONENT]];
    uint8_t *V[cm->padw[V_COMPONENT]*cm->padh[V_COMPONENT]];
  } *local_img_seg;
  
  //another struct of image segment to get the result segment from tegra back to x86
  volatile struct result_img_segment
  {
    int keyframe;
    struct macroblock *mbs[COLOR_COMPONENTS][cm->mb_rows * cm->mb_cols];
    /*residuals */
    int16_t *Ydct[cm->ypw * cm->yph]; 
    int16_t *Udct[cm->upw * cm->uph];
    int16_t *Vdct[cm->vpw * cm->vph];
  } *result_local_img_seg;

  /* file descriptor */
  SCIOpen(&v_dev,NO_FLAGS,&error);
  if (error != SCI_ERR_OK) {
     fprintf(stderr, "SCIOpen failed: %s (0x%x)\n",
             SCIGetErrorString(error), error);
     exit(error);
  }
  //PIO communication
  //create segment
  SCICreateSegment(v_dev,
                   &local_packets,
                   SEGMENT_LOCAL_COM,
                   sizeof(struct com_packets),
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
  SCIPrepareSegment(local_packets, local_adapter_num, NO_FLAGS, &error);
  if(error != SCI_ERR_OK){
    fprintf(stderr, "SCIPrepareSegment failed: %s - Error code: (0x%x)\n",
            SCIGetErrorString(error), error);
    exit(EXIT_FAILURE);
  }
  //set segment
  SCISetSegmentAvailable(local_packets, local_adapter_num, NO_FLAGS, &error);
  if(error != SCI_ERR_OK){
    fprintf(stderr, "SCISetSegmentAvailable failed: %s - Error code: (0x%x)\n",
            SCIGetErrorString(error), error);
    exit(EXIT_FAILURE);
  }

  //connect to tegra
  do {
      SCIConnectSegment(v_dev,
                        &remote_packets,
                        remote_node,
                        SEGMENT_REMOTE_COM,
                        local_adapter_num,
                        NO_CALLBACK,
                        NULL,
                        SCI_INFINITE_TIMEOUT,
                        NO_FLAGS,
                        &error);
  } while (error != SCI_ERR_OK);
  if(error != SCI_ERR_OK){
    fprintf(stderr, " failed: %s - Error code: (0x%x)\n",
            SCIGetErrorString(error), error);
    exit(EXIT_FAILURE);
  }

  //mapping remote and local segments for communication

  local_packets = SCIMapLocalSegment(local_segment_com,
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

  
  /*    Sending img width and img height to tegra with packets
  *   and set cmd==CMD_DONE so that it can stop waiting  */
  local_packets->packet.img_width = width;
  local_packets->packet.img_height = height;
  local_packets->packet.cmd = CMD_DONE;

  //create local segment for available image data
  SCICreateSegment(v_dev,
                   &local_segment,
                   SEGMENT_LOCAL,
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

  //preaper segment
  SCIPrepareSegment(local_segment, local_adapter_num, NO_FLAGS, &error);
  if(error != SCI_ERR_OK){
   fprintf(stderr, "SCIPrepareSegment failed: %s - Error code: (0x%x)\n",
           SCIGetErrorString(error), error);
   exit(EXIT_FAILURE);
  }
  //set segment
  SCISetSegmentAvailable(local_segment, local_adapter_num, NO_FLAGS, &error);
  if(error != SCI_ERR_OK){
   fprintf(stderr, "SCISetSegmentAvailable failed: %s - Error code: (0x%x)\n",
           SCIGetErrorString(error), error);
   exit(EXIT_FAILURE);
  }

  //create segment for results
  SCICreateSegment(v_dev,
                  &result_local_segment,
                  SEGMENT_LOCAL_RESULT,
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

  //prepare result segment
  SCIPrepareSegment(result_local_segment, local_adapter_num, NO_FLAGS, &error);
  if(error != SCI_ERR_OK){
    fprintf(stderr, "SCIPrepareSegment failed: %s - Error code: (0x%x)\n",
            SCIGetErrorString(error), error);
    exit(EXIT_FAILURE);
  }
  //set available segment
  SCISetSegmentAvailable(result_local_segment, local_adapter_num, NO_FLAGS, &error);
  if(error != SCI_ERR_OK){
    fprintf(stderr, "SCISetSegmentAvailable failed: %s - Error code: (0x%x)\n",
            SCIGetErrorString(error), error);
    exit(EXIT_FAILURE);
  }

  //Connecting to remote segment for the dma transfer of image data to tegra
  do {
     SCIConnectSegment(v_dev,
                       &remote_segment,
                       remote_node,
                       SEGMENT_REMOTE,
                       local_adapter_num,
                       NO_CALLBACK,
                       NULL,
                       SCI_INFINITE_TIMEOUT,
                       NO_FLAGS,
                       &error);
  } while (error != SCI_ERR_OK);

  //Mapping local segment for image transfer from x86
  local_img_seg =   SCIMapLocalSegment(local_segment,
                                      &local_map,
                                      0,
                                      sizeof(struct img_segment),
                                      NULL,
                                      NO_FLAGS,
                                      &error);
  if(error != SCI_ERR_OK){
    fprintf(stderr, "SCIMapLocalSegment failed: %s - Error code: (0x%x)\n",
            SCIGetErrorString(error), error);
    exit(EXIT_FAILURE);
  }

  //Mapping local segment for image to be encoded transfer from x86
  result_local_img_seg =   SCIMapLocalSegment(result_local_segment,
                                    &result_local_map,
                                    0,
                                    sizeof(struct result_img_segment),
                                    NULL,
                                    NO_FLAGS,
                                    &error);
  if(error != SCI_ERR_OK){
    fprintf(stderr, "SCIMapLocalSegment failed: %s - Error code: (0x%x)\n",
            SCIGetErrorString(error), error);
    exit(EXIT_FAILURE);
  }

  //Creating DMA queue for transfer
  SCICreateDMAQueue(v_dev,
                    &dmaq,
                    local_adapter_num,
                    maxEntries,
                    NO_FLAGS,
                    &error
                  );
  if(error != SCI_ERR_OK){
    fprintf(stderr, "SCICreateDMAQueue failed: %s - Error code: (0x%x)\n",
    SCIGetErrorString(error), error);
    exit(EXIT_FAILURE);
  }

  // create cm to write in c63_write
  cm->curframe = malloc(sizeof(struct frame));
  cm->curframe ->residuals = malloc(sizeof(dct_t));

  //Y
  cm->curframe ->residuals->Ydct = calloc(cm->ypw * cm->yph, sizeof(int16_t));
  //U
  cm->curframe ->residuals->Udct = calloc(cm->upw * cm->uph, sizeof(int16_t));
  //V
  cm->curframe ->residuals->Vdct = calloc(cm->vpw * cm->vph, sizeof(int16_t));

  //Memory allocation for Y,U,V components
  cm->curframe ->mbs[Y_COMPONENT] = calloc(cm->mb_rows * cm->mb_cols, sizeof(struct macroblock));
  cm->curframe ->mbs[U_COMPONENT] = calloc(cm->mb_rows/2 * cm->mb_cols/2, sizeof(struct macroblock));
  cm->curframe ->mbs[V_COMPONENT] = calloc(cm->mb_rows/2 * cm->mb_cols/2, sizeof(struct macroblock));


  int numframes = 0;
  // start time
  clock_gettime(CLOCK_MONOTONIC, &start_time);

   /* main loop, 
 ----read image 
 ----encode in tegra
 -----write frame */
  while (1)
  {
    //set CMD to INVALID
    local_packets->packet.cmd = CMD_INVALID;
    image = read_yuv(infile, cm);
    if (!image) { break; }

    //Copying memory blocks from image to client segment
    memcpy(local_img_seg->Y, image->Y, cm->padw[Y_COMPONENT]*cm->padh[Y_COMPONENT]);
    memcpy(local_img_seg->U, image->U, cm->padw[U_COMPONENT]*cm->padh[U_COMPONENT]);
    memcpy(local_img_seg->V, image->V, cm->padw[V_COMPONENT]*cm->padh[V_COMPONENT]);

    // Starting DMA transfer using DMA queue
    SCIStartDmaTransfer(dmaq,
                        local_segment,
                        remote_segment,
                        localOffset,
                        sizeof(struct img_segment),
                        remoteOffset,
                        NO_CALLBACK,
                        NULL,
                        NO_FLAGS,
                        &error);

    if(error != SCI_ERR_OK){
      fprintf(stderr, "SCIStartDmaTransfer failed: %s - Error code: (0x%x)\n",
      SCIGetErrorString(error), error);
      exit(EXIT_FAILURE);
    }

    // Waiting for DMA to finish transfer
    SCIWaitForDMAQueue(dmaq,
                      SCI_INFINITE_TIMEOUT,
                      NO_FLAGS,
                      &error);
    if(error != SCI_ERR_OK){
      fprintf(stderr, "SCIWaitForDMAQueue failed: %s - Error code: (0x%x)\n",
      SCIGetErrorString(error), error);
      exit(EXIT_FAILURE);
    }
    printf("Encoding frame %d, ", numframes);
    
    //Telling Tegra to start encoding
    
    remote_packets->packet.cmd = CMD_DONE;

    // Waiting for Tegra to finish encoding
    while(local_packets->packet.cmd != CMD_DONE);


    /* Copying memory blocks from local segments which has recived encoding results from Tegra */
    cm->curframe->keyframe = result_local_img_seg->keyframe;

    // Copying Macroblocks
    memcpy( cm->curframe->mbs[Y_COMPONENT],result_local_img_seg->mbs[Y_COMPONENT],cm->mb_rows * cm->mb_cols * sizeof(struct macroblock));  //Y
    memcpy( cm->curframe->mbs[U_COMPONENT],result_local_img_seg->mbs[U_COMPONENT],cm->mb_rows/2 * cm->mb_cols/2 * sizeof(struct macroblock));  //U
    memcpy( cm->curframe->mbs[V_COMPONENT],result_local_img_seg->mbs[V_COMPONENT],cm->mb_rows/2 * cm->mb_cols/2 * sizeof(struct macroblock)); //V

    // Copying residuals
    memcpy( cm->curframe->residuals->Ydct,result_local_img_seg->Ydct,cm->ypw * cm->yph * sizeof(int16_t)); //Ydct
    memcpy( cm->curframe->residuals->Udct,result_local_img_seg->Udct,cm->upw * cm->uph * sizeof(int16_t));  //Udct
    memcpy( cm->curframe->residuals->Vdct,result_local_img_seg->Vdct,cm->vpw * cm->vph * sizeof(int16_t));  //Vdct

    // write_frame
    write_frame(cm);
    printf("Done!\n");
    ++numframes;
    if (limit_numframes && numframes >= limit_numframes) { 
      break; 
    }
}

  //tell tegra to quit
  remote_packets->packet.cmd = CMD_QUIT;

  clock_gettime(CLOCK_MONOTONIC, &end_time);
    
  /* print time */
  elapsed = (end_time.tv_sec - start_time.tv_sec) +(end_time.tv_nsec - start_time.tv_nsec)/1e9;
  printf("Completed in %.3fs. s\n",elapsed);
  
  //closing operations
  fclose(outfile);
  fclose(infile);
  SCITerminate();

  

  //int i, j;
  //for (i = 0; i < 2; ++i)
  //{
  //  printf("int freq[] = {");
  //  for (j = 0; j < ARRAY_SIZE(frequencies[i]); ++j)
  //  {
  //    printf("%d, ", frequencies[i][j]);
  //  }
  //  printf("};\n");
  //}

  return EXIT_SUCCESS;
}
