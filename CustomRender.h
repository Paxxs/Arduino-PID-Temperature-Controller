#ifndef _MY_RENDERER_P_H
#define _MY_RENDERER_P_H

#define PRINT_NUM_U8                                                    \
    display_->print((const __FlashStringHelper *)menu_item.get_name()); \
    display_->print(F(":"));                                            \
    display_->print(menu_item.get_value());

#include <U8g2lib.h>

#include <MenuSystem.h>
#include "ToggleMenuItem.h"
#include "UIntMenuItem.h"

class ToggleMenuItem;
class UIntMenuItem;

class CustomRender : public MenuComponentRenderer
{
public:
    CustomRender(U8G2_SSD1306_128X64_NONAME_2_HW_I2C *d, uint8_t lines);
    // override
    /*virtual */ void render(Menu const &menu) const;

    /*virtual */ void render_menu_item(MenuItem const &menu_item) const;
    /*virtual */ void render_back_menu_item(BackMenuItem const &menu_item) const;
    /*virtual */ void render_numeric_menu_item(NumericMenuItem const &menu_item) const;
    /*virtual */ void render_menu(Menu const &menu) const;
    /*virtual */ void render_toggle_menu_item(ToggleMenuItem const &menu_item) const;
    /*virtual */ void render_uint_menu_item(UIntMenuItem const &menu_item) const;

private:
    U8G2_SSD1306_128X64_NONAME_2_HW_I2C *display_;
    uint8_t lines_;
};

#endif