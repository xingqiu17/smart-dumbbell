#ifndef LCD_DISPLAY_H
#define LCD_DISPLAY_H

#include "display.h"

#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>
#include <font_emoji.h>

#include <atomic>
#include <unordered_map>

static const char* kActionNameZh[] = {
/* EX_UNKNOWN */ "未知动作",
/* EX_AIDBC   */ "哑铃弯举",
/* ……         */ /* 你可以继续补全 */
/* EX_DWC     */ "哑铃侧平举",
/* EX_DLR     */ "哑铃划船",
/* EX_DSP     */ "哑铃深蹲推举",
/* EX_45DBP   */ "45°哑铃卧推",
/* EX_IDBC    */ "反握哑铃弯举",
};

/* 小工具函数：越界时返回“未知动作” */
static inline const char* ActionName(int id)
{
    if (id < 0 || id >= (int)(sizeof(kActionNameZh)/sizeof(kActionNameZh[0])))
        return kActionNameZh[0];
    return kActionNameZh[id];
}

// Theme color structure
struct ThemeColors {
    lv_color_t background;
    lv_color_t text;
    lv_color_t chat_background;
    lv_color_t user_bubble;
    lv_color_t assistant_bubble;
    lv_color_t system_bubble;
    lv_color_t system_text;
    lv_color_t border;
    lv_color_t low_battery;
};


class LcdDisplay : public Display {
protected:
    esp_lcd_panel_io_handle_t panel_io_ = nullptr;
    esp_lcd_panel_handle_t panel_ = nullptr;
    
    lv_draw_buf_t draw_buf_;
    lv_obj_t* status_bar_ = nullptr;
    lv_obj_t* content_ = nullptr;
    lv_obj_t* container_ = nullptr;

    //newf
    std::unordered_map<std::string, lv_obj_t*> pages_;
    lv_obj_t* current_page_ = nullptr;
    lv_obj_t* side_bar_ = nullptr;
    lv_obj_t* preview_image_ = nullptr;
    lv_obj_t* workout_page_ = nullptr;
    lv_obj_t* workout_name_label_ = nullptr;
    lv_obj_t* workout_count_label_ = nullptr;
    lv_obj_t* workout_score_label_ = nullptr;

        /* -------- pause 页面及子控件（预声明） -------- */
    lv_obj_t* pause_page_        = nullptr;   // 整个页面
    lv_obj_t* pause_action_lbl_  = nullptr;   // 动作名称（小字）
    lv_obj_t* pause_reps_lbl_    = nullptr;   // 目标次数（小字）
    lv_obj_t* pause_timer_lbl_   = nullptr;   // 倒计时（大字）

    esp_timer_handle_t pause_timer_handle_ = nullptr;
    int  pause_remaining_secs_ = 0;



    DisplayFonts fonts_;
    ThemeColors current_theme_;

    void SetupUI();
    virtual bool Lock(int timeout_ms = 0) override;
    virtual void Unlock() override;

protected:
    // 添加protected构造函数
    LcdDisplay(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_handle_t panel, DisplayFonts fonts, int width, int height);
    
public:
    ~LcdDisplay();
    virtual void SetEmotion(const char* emotion) override;
    virtual void SetIcon(const char* icon) override;
    virtual void SetPreviewImage(const lv_img_dsc_t* img_dsc) override;
#if CONFIG_USE_WECHAT_MESSAGE_STYLE
    virtual void SetChatMessage(const char* role, const char* content) override; 
#endif  

    // Add theme switching function
    virtual void SetTheme(const std::string& theme_name) override;
    // 页面管理
    virtual lv_obj_t* CreatePage(const std::string& id) override;
    virtual void ShowPage(const std::string& id) override;
    virtual void UpdateExercise(const std::string& name, int count,int count1, float score) override;
    virtual void UpdatePause(int action_id, int target_reps, int seconds)override;   // ← 新接口
    virtual void StopPause()override;                                               // 可选：提前终止
    
};

// RGB LCD显示器
class RgbLcdDisplay : public LcdDisplay {
public:
    RgbLcdDisplay(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_handle_t panel,
                  int width, int height, int offset_x, int offset_y,
                  bool mirror_x, bool mirror_y, bool swap_xy,
                  DisplayFonts fonts);
};

// MIPI LCD显示器
class MipiLcdDisplay : public LcdDisplay {
public:
    MipiLcdDisplay(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_handle_t panel,
                   int width, int height, int offset_x, int offset_y,
                   bool mirror_x, bool mirror_y, bool swap_xy,
                   DisplayFonts fonts);
};

// // SPI LCD显示器
class SpiLcdDisplay : public LcdDisplay {
public:
    SpiLcdDisplay(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_handle_t panel,
                  int width, int height, int offset_x, int offset_y,
                  bool mirror_x, bool mirror_y, bool swap_xy,
                  DisplayFonts fonts);
};

// QSPI LCD显示器
class QspiLcdDisplay : public LcdDisplay {
public:
    QspiLcdDisplay(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_handle_t panel,
                   int width, int height, int offset_x, int offset_y,
                   bool mirror_x, bool mirror_y, bool swap_xy,
                   DisplayFonts fonts);
};

// MCU8080 LCD显示器
class Mcu8080LcdDisplay : public LcdDisplay {
public:
    Mcu8080LcdDisplay(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_handle_t panel,
                      int width, int height, int offset_x, int offset_y,
                      bool mirror_x, bool mirror_y, bool swap_xy,
                      DisplayFonts fonts);
};

#endif // LCD_DISPLAY_H
