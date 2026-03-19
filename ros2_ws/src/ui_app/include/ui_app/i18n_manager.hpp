#ifndef I18N_MANAGER_HPP
#define I18N_MANAGER_HPP

#include <QString>
#include <QSettings>

enum class AppLanguage { Chinese, English };

class I18nManager {
public:
    static AppLanguage currentLanguage() {
        QSettings settings("RobotApp", "UI");
        QString lang = settings.value("language", "zh").toString();
        return lang == "en" ? AppLanguage::English : AppLanguage::Chinese;
    }

    static void setLanguage(AppLanguage lang) {
        QSettings settings("RobotApp", "UI");
        settings.setValue("language", lang == AppLanguage::English ? "en" : "zh");
    }

    static bool isEnglish() {
        return currentLanguage() == AppLanguage::English;
    }
};

inline QString tr_ui(const char* zh, const char* en) {
    return I18nManager::isEnglish() ? QString::fromUtf8(en) : QString::fromUtf8(zh);
}

#endif // I18N_MANAGER_HPP
