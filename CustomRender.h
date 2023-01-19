#ifndef _MY_RENDERER_P_H
#define _MY_RENDERER_P_H

// #include <Wire.h>
// #include <Adafruit_GFX.h>
// #include <Adafruit_SSD1306.h>
#include <U8g2lib.h>

#include <MenuSystem.h>

class CustomRender : public MenuComponentRenderer
{
public:
    CustomRender(U8G2_SSD1306_128X64_NONAME_2_HW_I2C *d, uint8_t lines);
    // override
    virtual void render(Menu const &menu) const;

    virtual void render_menu_item(MenuItem const &menu_item) const;
    virtual void render_back_menu_item(BackMenuItem const &menu_item) const;
    virtual void render_numeric_menu_item(NumericMenuItem const &menu_item) const;
    virtual void render_menu(Menu const &menu) const;

private:
    U8G2_SSD1306_128X64_NONAME_2_HW_I2C *display_;
    uint8_t lines_;
    void u8Print(const char *str) const;
};

#endif