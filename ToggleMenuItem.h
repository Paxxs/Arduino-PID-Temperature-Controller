#ifndef _TOGGLE_MENUITEM_H
#define _TOGGLE_MENUITEM_H

#include <MenuSystem.h>
#include "CustomRender.h"

class CustomRender;

class ToggleMenuItem : public MenuItem
{
private:
    bool *state_;
    const char *on_string_;
    const char *off_string_;

public:
    /**
     * @brief Construct a new Toggle Menu Item object
     *
     * @param name Menu Item Name
     * @param select_fn 被选择回调函数 Callback
     * @param on_string 开状态显示文字
     * @param off_string 关状态显示文字
     * @param state 状态指针
     */
    ToggleMenuItem(const char *name, SelectFnPtr select_fn,
                   const char *on_string, const char *off_string, bool *state);

    void set_state(bool state);
    bool get_state() const;

    void toggle_state();

    // 用于render使用的
    const char *get_state_string() const;
    virtual void render(MenuComponentRenderer const &renderer) const;

protected:
    //! \copydoc MenuComponent:select
    virtual Menu *select();
};

#endif //_TOGGLE_MENUITEM_H