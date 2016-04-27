/*-------------------------------------------------------------------------
  Source code is based on https://github.com/adafruit/Adafruit_NeoMatrix
  replace internal use of NeoPixel library with CRGB array to use with FastLED

  modified:  Juergen Skrotzky (JorgenVikingGod@gmail.com)
  date:      2016/04/27
  -------------------------------------------------------------------------
  Original copyright & description below
  -------------------------------------------------------------------------
  Arduino library to control single and tiled matrices of WS2811- and
  WS2812-based RGB LED devices such as the Adafruit NeoPixel Shield or
  displays assembled from NeoPixel strips, making them compatible with
  the Adafruit_GFX graphics library.  Requires both the Adafruit_NeoPixel
  and Adafruit_GFX libraries.

  Written by Phil Burgess / Paint Your Dragon for Adafruit Industries.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing products
  from Adafruit!

  -------------------------------------------------------------------------
  This file is part of the Adafruit NeoMatrix library.

  NeoMatrix is free software: you can redistribute it and/or modify
  it under the terms of the GNU Lesser General Public License as
  published by the Free Software Foundation, either version 3 of
  the License, or (at your option) any later version.

  NeoMatrix is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with NeoMatrix.  If not, see
  <http://www.gnu.org/licenses/>.
  -------------------------------------------------------------------------*/

#include <FastLED.h>
#include <FastLEDMatrix.h>

#ifdef __AVR__
 #include <avr/pgmspace.h>
#elif defined(ESP8266)
 #include <pgmspace.h>
#else
 #ifndef pgm_read_byte
  #define pgm_read_byte(addr) (*(const unsigned char *)(addr))
 #endif
#endif

#ifndef _swap_uint16_t
#define _swap_uint16_t(a, b) { uint16_t t = a; a = b; b = t; }
#endif

// Constructor for single matrix:
FastLEDMatrix::FastLEDMatrix(int w, int h, uint8_t matrixType)
  : Adafruit_GFX(w, h), type(matrixType), matrixWidth(w), matrixHeight(h),
    tilesX(0), tilesY(0), remapFn(NULL) {
  struct CRGB p_LED[(matrixWidth * matrixHeight];
  m_LED = p_LED;
}

// Constructor for tiled matrices:
FastLEDMatrix::FastLEDMatrix(uint8_t mW, uint8_t mH, uint8_t tX, uint8_t tY, uint8_t matrixType)
  : Adafruit_GFX(mW * tX, mH * tY), type(matrixType), matrixWidth(mW), matrixHeight(mH),
    tilesX(tX), tilesY(tY), remapFn(NULL) {
  struct CRGB p_LED[(matrixWidth*tilesX * matrixHeight*tilesY];
  m_LED = p_LED;
}

struct CRGB* FastLEDMatrix::operator[](int n) {
  return(&m_LED[n]);
}

struct CRGB& FastLEDMatrix::operator()(int16_t x, int16_t y) {
  if ( (x >= 0) && (x < _width) && (y >= 0) && (y < _height))
    return(m_LED[mXY(x, y)]);
  else
    return(m_OutOfBounds);
}

struct CRGB& FastLEDMatrix::operator()(int16_t i) {
  if ((i >=0) && (i < (_width * _height)))
    return(m_LED[i]);
  else
    return(m_OutOfBounds);
}

void FastLEDMatrix::SetLEDArray(struct CRGB *pLED) {
  m_LED = pLED;
}

virtual uint16_t FastLEDMatrix::mXY(uint16_t x, uint16_t y) {
  if((x < 0) || (y < 0) || (x >= _width) || (y >= _height)) return 0;

  int16_t t;
  switch(rotation) {
   case 1:
    t = x;
    x = WIDTH  - 1 - y;
    y = t;
    break;
   case 2:
    x = WIDTH  - 1 - x;
    y = HEIGHT - 1 - y;
    break;
   case 3:
    t = x;
    x = y;
    y = HEIGHT - 1 - t;
    break;
  }

  int tileOffset = 0, pixelOffset;

  if(remapFn) { // Custom X/Y remapping function
    pixelOffset = (*remapFn)(x, y);
  } else {      // Standard single matrix or tiled matrices
    uint8_t  corner = type & MTX_MATRIX_CORNER;
    uint16_t minor, major, majorScale;

    if(tilesX) { // Tiled display, multiple matrices
      uint16_t tile;

      minor = x / matrixWidth;            // Tile # X/Y; presume row major to
      major = y / matrixHeight,           // start (will swap later if needed)
      x     = x - (minor * matrixWidth);  // Pixel X/Y within tile
      y     = y - (major * matrixHeight); // (-* is less math than modulo)

      // Determine corner of entry, flip axes if needed
      if(type & MTX_TILE_RIGHT)  minor = tilesX - 1 - minor;
      if(type & MTX_TILE_BOTTOM) major = tilesY - 1 - major;

      // Determine actual major axis of tiling
      if((type & MTX_TILE_AXIS) == MTX_TILE_ROWS) {
        majorScale = tilesX;
      } else {
        _swap_uint16_t(major, minor);
        majorScale = tilesY;
      }

      // Determine tile number
      if((type & MTX_TILE_SEQUENCE) == MTX_TILE_PROGRESSIVE) {
        // All tiles in same order
        tile = major * majorScale + minor;
      } else {
        // Zigzag; alternate rows change direction.  On these rows,
        // this also flips the starting corner of the matrix for the
        // pixel math later.
        if(major & 1) {
          corner ^= MTX_MATRIX_CORNER;
          tile = (major + 1) * majorScale - 1 - minor;
        } else {
          tile =  major      * majorScale     + minor;
        }
      }
      // Index of first pixel in tile
      tileOffset = tile * matrixWidth * matrixHeight;
    } // else no tiling (handle as single tile)

    // Find pixel number within tile
    minor = x; // Presume row major to start (will swap later if needed)
    major = y;

    // Determine corner of entry, flip axes if needed
    if(corner & MTX_MATRIX_RIGHT)  minor = matrixWidth  - 1 - minor;
    if(corner & MTX_MATRIX_BOTTOM) major = matrixHeight - 1 - major;

    // Determine actual major axis of matrix
    if((type & MTX_MATRIX_AXIS) == MTX_MATRIX_ROWS) {
      majorScale = matrixWidth;
    } else {
      _swap_uint16_t(major, minor);
      majorScale = matrixHeight;
    }

    // Determine pixel number within tile/matrix
    if((type & MTX_MATRIX_SEQUENCE) == MTX_MATRIX_PROGRESSIVE) {
      // All lines in same order
      pixelOffset = major * majorScale + minor;
    } else {
      // Zigzag; alternate rows change direction.
      if(major & 1) pixelOffset = (major + 1) * majorScale - 1 - minor;
      else          pixelOffset =  major      * majorScale     + minor;
    }
  }
  return (tileOffset + pixelOffset);
}

// Downgrade 24-bit color to 16-bit (add reverse gamma lookup here?)
uint16_t FastLEDMatrix::Color(CRGB color) {
  return ((uint16_t)(color.r & 0xF8) << 8) |
         ((uint16_t)(color.g & 0xFC) << 3) |
                    (color.b         >> 3);
}

void FastLEDMatrix::drawPixel(int16_t x, int16_t y, CRGB color) {
  m_LED[mXY(x,y)] = color;
}

void FastLEDMatrix::fillScreen(uint16_t color) {
  uint16_t i, n;
  uint32_t c;

  n = numPixels();
  for(i=0; i<n; i++) m_LED[i] = c;
}

void FastLEDMatrix::setRemapFunction(uint16_t (*fn)(uint16_t, uint16_t)) {
  remapFn = fn;
}
