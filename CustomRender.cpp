#include "CustomRender.h"
#include "logo_xbm.h"

CustomRender::CustomRender(U8G2_SSD1306_128X64_NONAME_2_HW_I2C *d, uint8_t lines) : display_(d), lines_(lines)
{
}

void CustomRender::u8Print(const char *str) const
{
    char c;
    if (!str)
        return;
    while ((c = pgm_read_byte(str++)))
    {
        display_->print(c);
    }
}

void CustomRender::render(Menu const &menu) const
{
    display_->firstPage();
    do
    {
        display_->setFont(u8g2_font_t0_11_tr);
        display_->setFontPosTop(); // 顶部对齐
        display_->setDrawColor(1); // 不反色
        display_->drawXBMP(128 - 2, 0, scrollbar_background_small_width, scrollbar_background_small_height, scrollbar_background_small_bits);
        // 绘制滚动条
        display_->drawBox(125, 64 / menu.get_num_components() * menu.get_current_component_num(), 3, 64 / menu.get_num_components());
        display_->setCursor(8, 3);

        // Serial.print(F("get_current_component_num()="));
        // Serial.println(menu.get_current_component_num());
        // Serial.print(F("get_num_components()="));
        // Serial.println(menu.get_num_components());
        // Serial.print(F("first_display_line = "));

        uint8_t first_display_line = menu.get_current_component_num() /* 当前选择的id*/ - menu.get_current_component_num() % lines_;

        // Serial.print(F("64 / menu.get_num_components() * menu.get_current_component_num()="));
        // Serial.println(64 / menu.get_num_components() * menu.get_current_component_num());
        // Serial.print(F("64 / menu.get_num_components()"));
        // Serial.println(64 / menu.get_num_components());
        for (uint8_t i = first_display_line; ((/*不超出 lines 限制*/ i < first_display_line + lines_) && /*不超过总数*/ i < menu.get_num_components()); ++i)
        {
            uint8_t index = (i % lines_) /*当前index*/ * 16; // 简化计算量
            MenuComponent const *menu_component = menu.get_menu_component(i /*这里i是菜单list的索引*/);

            // Serial.print(i);

            if (menu_component->is_current())
            {

                // Serial.print(F("[当前]"));

                display_->setDrawColor(1); // 重置绘制模式，绘制前景不背景
                display_->drawRBox(0, /*避免超出*/ index, 128 - 1 - scrollbar_background_width /* 留进度条*/, 16, 2);
                display_->setDrawColor(0); // 只绘制背景
                if (menu_component->has_focus())
                {
                    display_->setFont(u8g2_font_open_iconic_embedded_1x_t);
                    // display_->drawGlyph(0, index + 4, 69);
                    display_->print('H');
                    display_->setFont(u8g2_font_t0_12b_tr);
                    display_->print(' ');
                    // Serial.print(F("[选中]"));
                }
                display_->setFont(u8g2_font_t0_12b_tr);
            }
            else
            {
                display_->setFont(u8g2_font_t0_11_tr);
                display_->setDrawColor(1); // 不制背景
                // Serial.print(F("[普通]"));
            }

            menu_component->render(*this);
            display_->setCursor(8, 19 + index);
            // Serial.print(F(" "));
        }
        // Serial.println(F("===="));
    } while (display_->nextPage());
}

void CustomRender::render_menu_item(MenuItem const &menu_item) const
{
    // display_->print(menu_item.get_name());
    u8Print(menu_item.get_name());
}

void CustomRender::render_back_menu_item(BackMenuItem const &menu_item) const
{
    // display_->print(menu_item.get_name());
    u8Print(menu_item.get_name());
}

void CustomRender::render_numeric_menu_item(NumericMenuItem const &menu_item) const
{
    // display_->print(menu_item.get_name());
    PRINT_NUM_U8;
}

void CustomRender::render_uint_menu_item(UIntMenuItem const &menu_item) const
{
    PRINT_NUM_U8;
}

void CustomRender::render_toggle_menu_item(ToggleMenuItem const &menu_item) const
{
    u8Print(menu_item.get_name());
    display_->print(F(":"));
    u8Print(menu_item.get_state_string());
}

void CustomRender::render_menu(Menu const &menu) const
{
    // display_->print(menu.get_name());
    u8Print(menu.get_name());
}