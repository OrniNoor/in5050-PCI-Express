#ifndef C63_COMMON_H_
#define C63_COMMON_H_
#include <inttypes.h>
#include <stdint.h>
#include "c63.h"

// For SISCI
#define GROUP 8
#ifndef GROUP
#error Fill in group number in common.h!
#endif
#define GET_SEGMENTID(id) ( GROUP << 16 | id )

#define NO_CALLBACK NULL
#define NO_FLAGS 0

/* Segment IDs for transfer */
#define SEGMENT_LOCAL GET_SEGMENTID(1)
#define SEGMENT_REMOTE GET_SEGMENTID(2)

/* Segment IDs for PIO */ 
#define SEGMENT_LOCAL_COM GET_SEGMENTID(3)
#define SEGMENT_REMOTE_COM GET_SEGMENTID(4)

// Segment for encoded results
#define SEGMENT_LOCAL_RESULT GET_SEGMENTID(5)
#define SEGMENT_REMOTE_RESULT GET_SEGMENTID(6)


// Commands for communication
enum cmd
{
    CMD_INVALID,    //used to tell to wait
    CMD_QUIT,       //used to tell to stop waiting 
    CMD_DONE        //used to exit from operation
};

//data packet with image params
struct packet
{
  union {
    struct{
      uint8_t cmd;
      int img_width;
      int img_height;
    };
  };
};

//used to transfer packets containg image data
struct com_packets {
  struct packet packet;
};



struct frame* create_frame(struct c63_common *cm, yuv_t *image);

void dct_quantize(uint8_t *in_data, uint8_t *prediction, uint32_t width,
    uint32_t height, int16_t *out_data, uint8_t *quantization);

void dequantize_idct(int16_t *in_data, uint8_t *prediction, uint32_t width,
    uint32_t height, uint8_t *out_data, uint8_t *quantization);

void destroy_frame(struct frame *f);

void dump_image(yuv_t *image, int w, int h, FILE *fp);

#endif  /* C63_COMMON_H_ */
