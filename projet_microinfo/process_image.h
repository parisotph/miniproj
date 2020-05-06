#ifndef PROCESS_IMAGE_H
#define PROCESS_IMAGE_H

float get_distance_cm(void);
uint16_t get_line_position(void);
uint8_t get_line_situation(void);
uint8_t get_target_situation(void);

void reset_pursuit(void);
void process_image_start(void);

#endif /* PROCESS_IMAGE_H */
