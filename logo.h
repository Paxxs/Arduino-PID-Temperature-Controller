#define SSD1306_NO_SPLASH 0  // nosplash

#define logo16_width  16
#define logo16_height 16

const uint8_t PROGMEM logo16_data[] = {
  0b00000000,0b00000000,
  0b01111000,0b00011110,
  0b01111000,0b00011110,
  0b01111000,0b00011110,
  0b00110011,0b11101110,
  0b00111011,0b11111110,
  0b00111111,0b01111100,
  0b00011111,0b11111000,
  0b01111111,0b11111110,
  0b11111111,0b11111111,
  0b11111111,0b11101111,
  0b11111111,0b11111111,
  0b11111111,0b01111111,
  0b01110000,0b00001110,
  0b00000000,0b00000000,
  0b00000000,0b00000000,
};



#define logo_width  82
#define logo_height 64

const uint8_t PROGMEM logo_data[] = {
  0b00000001,0b00000000,0b00000010,0b00011000,0b00011110,0b11111000,0b01111111,0b11100000,0b00000000,0b00000000,0b00000000,
  0b00000011,0b11101111,0b10001111,0b10111110,0b01110100,0b01001000,0b01000000,0b01100000,0b00000000,0b00000000,0b00000000,
  0b00000110,0b11001001,0b10000010,0b00011000,0b00111111,0b11111000,0b01111111,0b11100000,0b00000000,0b00000000,0b00000000,
  0b00000011,0b11111001,0b10001111,0b11111110,0b00101010,0b10100000,0b01000010,0b00000000,0b00000000,0b00000000,0b00000000,
  0b00000000,0b11001111,0b10001111,0b10011000,0b00111110,0b11111000,0b01111111,0b11100000,0b00000000,0b00000000,0b00000000,
  0b00000001,0b11111111,0b10000111,0b11111100,0b00101011,0b10100000,0b01111111,0b11100000,0b00000000,0b00000000,0b00000000,
  0b00000111,0b00100000,0b00000000,0b00000100,0b01111110,0b11111000,0b01000010,0b00000000,0b00011000,0b00011000,0b00000000,
  0b00000001,0b11111111,0b00000111,0b11111100,0b00000111,0b00100000,0b01011111,0b11100010,0b01001000,0b00100100,0b00000000,
  0b00000001,0b10000011,0b00000000,0b00000100,0b00011111,0b11110000,0b11011111,0b11100010,0b01001000,0b00100100,0b00000000,
  0b00000001,0b11111111,0b00000111,0b11111100,0b00011111,0b11110000,0b11011000,0b01100001,0b01001000,0b00100100,0b00000000,
  0b00000001,0b10000011,0b00000101,0b00000100,0b00000001,0b00011000,0b11011111,0b11100001,0b10001000,0b00100100,0b00000000,
  0b00000001,0b11111111,0b00011101,0b11111011,0b00111111,0b11111101,0b10011111,0b11100000,0b10011100,0b10011000,0b00000000,
  0b00011111,0b11111111,0b11111111,0b11111111,0b11111111,0b11111111,0b11111111,0b11111111,0b11111111,0b11111110,0b00000000,
  0b00111111,0b11111111,0b11111111,0b11111111,0b11111111,0b11111111,0b11111111,0b11111111,0b11111111,0b11111111,0b00000000,
  0b00111111,0b11101101,0b01000000,0b01111100,0b11010101,0b01111100,0b01000110,0b01110100,0b11001000,0b01111111,0b00000000,
  0b00111111,0b11101101,0b01011110,0b11111101,0b11010101,0b01111101,0b01010101,0b10110101,0b10111110,0b11111111,0b00000000,
  0b00111111,0b11101101,0b01011110,0b11111101,0b11010101,0b01111101,0b01010101,0b10110101,0b10111110,0b11111111,0b00000000,
  0b00111111,0b11101101,0b01101110,0b11111100,0b11010101,0b01111101,0b01001101,0b10110100,0b10111110,0b11111111,0b00000000,
  0b00111111,0b11101101,0b01110110,0b11111101,0b11010100,0b01111100,0b11001101,0b10110101,0b10111110,0b11111111,0b00000000,
  0b00111111,0b11101101,0b01110110,0b11111101,0b11010101,0b01111101,0b11010101,0b10110101,0b10111110,0b11111111,0b00000000,
  0b00111111,0b11001100,0b01000110,0b11111101,0b11000101,0b01111101,0b11010110,0b00100100,0b11001110,0b11111111,0b00000000,
  0b00111111,0b11111111,0b11111111,0b11111111,0b11111111,0b11111111,0b11111111,0b11111111,0b11111111,0b11111111,0b00000000,
  0b00011111,0b11111111,0b11111111,0b11111111,0b11111111,0b11111111,0b11111111,0b11111111,0b11111111,0b11111110,0b00000000,
  0b00000000,0b00000000,0b00111111,0b11111100,0b00000000,0b00000000,0b00000111,0b11111111,0b10000000,0b00000000,0b00000000,
  0b00000000,0b00000000,0b00111111,0b11111100,0b00000000,0b00000000,0b00000111,0b11111111,0b10000000,0b00000000,0b00000000,
  0b00000000,0b00000000,0b00111111,0b11111100,0b00000000,0b00000000,0b00000111,0b11111111,0b10000000,0b00000000,0b00000000,
  0b00000000,0b00000000,0b00111111,0b11111100,0b00000000,0b00000000,0b00000111,0b11111111,0b10000000,0b00000000,0b00000000,
  0b00000000,0b00000000,0b00111111,0b11111100,0b00000000,0b00000000,0b00000111,0b11111111,0b10000000,0b00000000,0b00000000,
  0b00000000,0b00000000,0b00111111,0b11111100,0b00000000,0b00000000,0b00000111,0b11111111,0b10000000,0b00000000,0b00000000,
  0b00000000,0b00000000,0b00111111,0b11111100,0b00000000,0b00000000,0b00000111,0b11111111,0b10000000,0b00000000,0b00000000,
  0b00000000,0b00000000,0b00000111,0b11100000,0b00000000,0b00000000,0b00000000,0b11111110,0b00000000,0b00000000,0b00000000,
  0b00000000,0b00000000,0b00000111,0b11100000,0b00000000,0b00000000,0b00000000,0b11111110,0b00000000,0b00000000,0b00000000,
  0b00000000,0b00000000,0b00000111,0b11100000,0b00000000,0b00000000,0b00000000,0b11111110,0b00000000,0b00000000,0b00000000,
  0b00000000,0b00000000,0b00000111,0b11100000,0b00011110,0b00001111,0b00000000,0b11111110,0b00000000,0b00000000,0b00000000,
  0b00000000,0b00000000,0b00000111,0b11110000,0b00011111,0b00011111,0b00000001,0b11111110,0b00000000,0b00000000,0b00000000,
  0b00000000,0b00000000,0b00000111,0b11111000,0b00011111,0b00011111,0b10000011,0b11111110,0b00000000,0b00000000,0b00000000,
  0b00000000,0b00000000,0b00000111,0b11111100,0b00011111,0b00011111,0b00000111,0b11111100,0b00000000,0b00000000,0b00000000,
  0b00000000,0b00000000,0b00000011,0b11111110,0b00011110,0b00001111,0b00001111,0b11111100,0b00000000,0b00000000,0b00000000,
  0b00000000,0b00000000,0b00000001,0b11111111,0b00001100,0b00000110,0b00011111,0b11111000,0b00000000,0b00000000,0b00000000,
  0b00000000,0b00000000,0b00000000,0b11111111,0b10001100,0b00000110,0b00011111,0b11110000,0b00000000,0b00000000,0b00000000,
  0b00000000,0b00000000,0b00000000,0b01111111,0b10001100,0b00000110,0b00111111,0b11100000,0b00000000,0b00000000,0b00000000,
  0b00000000,0b00000000,0b00000000,0b00111111,0b10001111,0b11111110,0b00111111,0b11000000,0b00000000,0b00000000,0b00000000,
  0b00000000,0b00000000,0b00000000,0b00111111,0b10001111,0b11111110,0b00111111,0b10000000,0b00000000,0b00000000,0b00000000,
  0b00000000,0b00000000,0b00000000,0b00111111,0b10001111,0b11111110,0b00111111,0b10000000,0b00000000,0b00000000,0b00000000,
  0b00000000,0b00000000,0b00000000,0b00111111,0b10000000,0b00000000,0b00111111,0b10000000,0b00000000,0b00000000,0b00000000,
  0b00000000,0b00000000,0b00000000,0b00111111,0b11111100,0b00000111,0b11111111,0b10000000,0b00000000,0b00000000,0b00000000,
  0b00000000,0b00000000,0b00111111,0b11111111,0b11111110,0b00001111,0b11111111,0b11111111,0b00000000,0b00000000,0b00000000,
  0b00000000,0b00000000,0b01111111,0b11111111,0b11111110,0b00001111,0b11111111,0b11111111,0b10000000,0b00000000,0b00000000,
  0b00000000,0b00000000,0b11111111,0b11111111,0b11111111,0b00011111,0b11111111,0b11111111,0b11000000,0b00000000,0b00000000,
  0b00000000,0b00000001,0b11110000,0b00011111,0b11111111,0b10111111,0b11111111,0b11111011,0b11100000,0b00000000,0b00000000,
  0b00000000,0b00000001,0b11101111,0b11111111,0b11111111,0b11111111,0b11111111,0b11111101,0b11100000,0b00000000,0b00000000,
  0b00000000,0b00000011,0b11011111,0b11111111,0b11111111,0b11111111,0b10000001,0b11111110,0b11110000,0b00000000,0b00000000,
  0b00000000,0b00000011,0b10111100,0b00011111,0b11111111,0b11111111,0b10000000,0b00001111,0b01110000,0b00000000,0b00000000,
  0b00000000,0b00000011,0b01111011,0b11111111,0b11111111,0b11111111,0b10000001,0b11110111,0b10110000,0b00000000,0b00000000,
  0b00000000,0b00000011,0b11111111,0b11111111,0b11111101,0b11110111,0b10000001,0b11111111,0b11110000,0b00000000,0b00000000,
  0b00000000,0b00000011,0b11111111,0b11111101,0b11111100,0b11100111,0b10000001,0b11111111,0b11110000,0b00000000,0b00000000,
  0b00000000,0b00000011,0b11111110,0b11111101,0b11111100,0b01000111,0b10000001,0b11011111,0b11110000,0b00000000,0b00000000,
  0b00000000,0b00000011,0b11111101,0b11111101,0b11111100,0b00000111,0b10000001,0b11101111,0b11110000,0b00000000,0b00000000,
  0b00000000,0b00000011,0b11111111,0b11111100,0b01111100,0b00000111,0b11111111,0b11111111,0b11110000,0b00000000,0b00000000,
  0b00000000,0b00000000,0b11111111,0b10011100,0b01111100,0b00000111,0b11111110,0b01111111,0b11000000,0b00000000,0b00000000,
  0b00000000,0b00000000,0b11111111,0b10011100,0b01111100,0b00000111,0b11111110,0b01111111,0b11000000,0b00000000,0b00000000,
  0b00000000,0b00000000,0b00011111,0b10000000,0b00000000,0b00000000,0b00000000,0b01111110,0b00000000,0b00000000,0b00000000,
  0b00000000,0b00000000,0b00011111,0b10000000,0b00000000,0b00000000,0b00000000,0b01111110,0b00000000,0b00000000,0b00000000,
  0b00000000,0b00000000,0b00000011,0b10000000,0b00000000,0b00000000,0b00000000,0b01110000,0b00000000,0b00000000,0b00000000,
};
