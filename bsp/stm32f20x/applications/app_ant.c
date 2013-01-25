#include <rtgui/rtgui.h>
#include <rtgui/rtgui_system.h>
#include <rtgui/rtgui_app.h>

#include <rtgui/widgets/window.h>
#include <rtgui/widgets/notebook.h>

struct rtgui_notebook *the_notebook;

static rt_bool_t demo_handle_key(struct rtgui_object *object, struct rtgui_event *event)
{
    struct rtgui_event_kbd *ekbd = (struct rtgui_event_kbd *)event;

    if (ekbd->type == RTGUI_KEYUP)
    {
        if (ekbd->key == RTGUIK_RIGHT)
        {
            demo_view_next(RT_NULL, RT_NULL);
            return RT_TRUE;
        }
        else if (ekbd->key == RTGUIK_LEFT)
        {
            demo_view_prev(RT_NULL, RT_NULL);
            return RT_TRUE;
        }
    }
    return RT_TRUE;
}

struct rtgui_win *main_win;
static void app_ant_entry(void *parameter)
{
    struct rtgui_app *app;
    struct rtgui_rect rect;

    app = rtgui_app_create(rt_thread_self(), "gui_ant");
    if (app == RT_NULL)
        return;

    /* create a full screen window */
    rtgui_graphic_driver_get_rect(rtgui_graphic_driver_get_default(), &rect);

    main_win = rtgui_win_create(RT_NULL, "ant_win", &rect,
                                RTGUI_WIN_STYLE_NO_BORDER | RTGUI_WIN_STYLE_NO_TITLE);
    if (main_win == RT_NULL)
    {
        rtgui_app_destroy(app);
        return;
    }

    rtgui_win_set_onkey(main_win, demo_handle_key);

    /* create a no title notebook that we can switch demo on it easily. */
    the_notebook = rtgui_notebook_create(&rect, RTGUI_NOTEBOOK_NOTAB);
    if (the_notebook == RT_NULL)
    {
        rtgui_win_destroy(main_win);
        rtgui_app_destroy(app);
        return;
    }

    rtgui_container_add_child(RTGUI_CONTAINER(main_win), RTGUI_WIDGET(the_notebook));

    demo_view_dc();
#ifdef RTGUI_USING_TTF
    demo_view_ttf();
#endif

#ifndef RTGUI_USING_SMALL_SIZE
    demo_view_dc_buffer();
#endif
    demo_view_animation();
#ifndef RTGUI_USING_SMALL_SIZE
    demo_view_buffer_animation();
    demo_view_instrument_panel();
#endif
    demo_view_window();
    demo_view_label();
    demo_view_button();
    demo_view_checkbox();
    demo_view_progressbar();
    demo_view_scrollbar();
    demo_view_radiobox();
    demo_view_textbox();
    demo_view_listbox();
    demo_view_menu();
    //demo_view_listctrl();
    demo_view_combobox();
    demo_view_slider();
    demo_view_notebook();
    demo_view_mywidget();
    demo_plot();

#if defined(RTGUI_USING_DFS_FILERW)
	demo_view_edit();
	demo_view_bmp();
#endif

    rtgui_win_show(main_win, RT_FALSE);

    rtgui_app_run(app);

    rtgui_app_destroy(app);
}

void app_ant()
{
    static rt_bool_t launched = RT_FALSE;

    if (launched == RT_FALSE)
    {
        rt_thread_t tid;

        tid = rt_thread_create("app_ant",
                               app_ant_entry, RT_NULL,
                               2048 * 2, 25, 10);

        if (tid != RT_NULL)
            rt_thread_startup(tid);

        launched = RT_TRUE;
    }
}

#ifdef RT_USING_FINSH
#include <finsh.h>
void app_ant()
{
    app_ant();
}
/* finsh application() */
FINSH_FUNCTION_EXPORT(app_ant, service for ANT+)
#endif
