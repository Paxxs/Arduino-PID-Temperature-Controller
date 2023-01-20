#include "UIntMenuItem.h"

UIntMenuItem::UIntMenuItem(const char *name, SelectFnPtr select_fn,
                           uint32_t value, uint32_t max_value,
                           uint32_t increment)
    : MenuItem(name, select_fn), value_(value), max_value_(max_value),
      increment_(increment)
{
}

Menu *UIntMenuItem::select()
{
    _has_focus = !_has_focus;

    // 编辑完成后才执行 callback
    if (!_has_focus && _select_fn != nullptr)
        _select_fn(this);
    return nullptr;
}

bool UIntMenuItem::next(bool loop)
{

    value_ += increment_;
    if (get_value() > max_value_)
    {
        if (loop) // 如果循环，超过最大则最小
            value_ = 0;
        else
            value_ = max_value_;
    }
    return true;
}
bool UIntMenuItem::prev(bool loop)
{
    value_ -= increment_;
    if (get_value() < 0)
    {
        if (loop) // 如果循环，小于最小则最大
            value_ = max_value_;
        else
            value_ = 0;
    }
    return true;
}

uint32_t UIntMenuItem::get_value() const
{
    return value_;
}
uint32_t UIntMenuItem::get_max_value() const
{
    return max_value_;
}
uint32_t UIntMenuItem::get_min_value() const
{
    return 0;
}

void UIntMenuItem::set_value(uint32_t value)
{

    value_ = value;
}
void UIntMenuItem::set_max_value(uint32_t value)
{
    max_value_ = value;
}

// 渲染
void UIntMenuItem::render(MenuComponentRenderer const &renderer) const
{
    CustomRender const &custom_renderer = static_cast<CustomRender const &>(renderer);
    custom_renderer.render_uint_menu_item(*this);
}
