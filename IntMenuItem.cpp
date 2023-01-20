#include "IntMenuItem.h"

IntMenuItem::IntMenuItem(const char *name, SelectFnPtr select_fn,
                         uint32_t value, uint32_t min_value, uint32_t max_value,
                         uint32_t increment)
    : MenuItem(name, select_fn), value_(value), min_value_(min_value), max_value_(max_value),
      increment_(increment)
{
    // 防呆设计
    if (min_value_ > max_value_)
    {
        uint32_t tmp = max_value_;
        max_value_ = min_value_;
        min_value_ = tmp;
    }
}

Menu *IntMenuItem::select()
{
    _has_focus = !_has_focus;

    // 编辑完成后才执行 callback
    if (!_has_focus && _select_fn != nullptr)
        _select_fn(this);
    return nullptr;
}

bool IntMenuItem::next(bool loop)
{
    value_ += increment_;
    if (value_ > max_value_)
    {
        if (loop) // 如果循环，超过最大则最小
            set_value(min_value_);
        else
            set_value(max_value_);
    }
    return true;
}
bool IntMenuItem::prev(bool loop)
{
    value_ -= increment_;
    if (value_ > min_value_)
    {
        if (loop) // 如果循环，小于最小则最大
            set_value(max_value_);
        else
            set_value(min_value_);
    }
    return true;
}

uint32_t IntMenuItem::get_value() const
{
    return value_;
}
uint32_t IntMenuItem::get_max_value() const
{
    return max_value_;
}
uint32_t IntMenuItem::get_min_value() const
{
    return min_value_;
}

void IntMenuItem::set_value(uint32_t value)
{
    value_ = value;
}
void IntMenuItem::set_max_value(uint32_t value)
{
    max_value_ = value;
}
void IntMenuItem::set_min_value(uint32_t value)
{
    min_value_ = value;
}

// 渲染
void IntMenuItem::render(MenuComponentRenderer const &renderer) const
{
    CustomRender const &custom_renderer = static_cast<CustomRender const &>(renderer);
    custom_renderer.render_uint_menu_item(*this);
}
