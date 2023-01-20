#ifndef _NUM_INT_MENUITEM_H
#define _NUM_INT_MENUITEM_H

#include <MenuSystem.h>
#include "CustomRender.h"

class CustomRender;

class IntMenuItem : public MenuItem
{
private:
    uint32_t value_;
    uint32_t min_value_;
    uint32_t max_value_;
    uint32_t increment_;

public:
    IntMenuItem(const char *name, SelectFnPtr select_fn,
                uint32_t value, uint32_t min_value, uint32_t max_value,
                uint32_t increment = 1);
    uint32_t get_value() const;
    uint32_t get_max_value() const;
    uint32_t get_min_value() const;

    void set_value(uint32_t value);
    void set_max_value(uint32_t value);
    void set_min_value(uint32_t value);

    virtual void render(MenuComponentRenderer const &renderer) const;

protected:
    virtual bool next(bool loop = false);
    virtual bool prev(bool loop = false);

    virtual Menu *select();
};

#endif // _NUM_INT_MENUITEM_H
