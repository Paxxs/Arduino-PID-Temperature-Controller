#include "ToggleMenuItem.h"

ToggleMenuItem::ToggleMenuItem(const char *name, SelectFnPtr select_fn, const char *on_string, const char *off_string, bool *state)
    : MenuItem(name, select_fn), on_string_(on_string), off_string_(off_string), state_(state)
{
}

void ToggleMenuItem::set_state(bool state)
{
    *state_ = state;
}

bool ToggleMenuItem::get_state() const
{
    return *state_;
}

void ToggleMenuItem::toggle_state()
{
    *state_ = !*state_;
}

const char *ToggleMenuItem::get_state_string() const
{
    if (get_state())
        return on_string_;
    return off_string_;
}

Menu *ToggleMenuItem::select()
{
    // 切换状态
    toggle_state();
    // 如果有被选择回调函数则执行
    if (_select_fn != nullptr)
        _select_fn(this);
    // return MenuComponent::select();
    return nullptr;
}

void ToggleMenuItem::render(MenuComponentRenderer const &renderer) const
{
    CustomRender const &custom_renderer = static_cast<CustomRender const &>(renderer);
    custom_renderer.render_toggle_menu_item(*this);
}
