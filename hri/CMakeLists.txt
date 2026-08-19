cmake_minimum_required(VERSION 3.16)
project(hri LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)

# Qt6: Quick/QML(主界面) + Widgets(暂留作后备) + Network(HTTP/WS 客户端) + QuickControls2(QQuickStyle)
find_package(Qt6 REQUIRED COMPONENTS Core Gui Quick Qml Widgets Network QuickControls2)

# qt_add_executable 自动完成 AUTOMOC/AUTORCC 等 Qt 集成
# MANUAL_FINALIZATION: 推迟 finalize, 由文件末尾的 qt_finalize_executable 统一完成 (避免重复 finalize 警告)
qt_add_executable(hri_app MANUAL_FINALIZATION
    src/main.cpp
    src/app/AppController.cpp
    src/app/AppController.h
    src/app/RobotClient.cpp
    src/app/RobotClient.h
    src/app/WsClient.cpp
    src/app/WsClient.h
)

# 让 QML 文件以相对模块根的方式嵌入资源 (否则资源路径会带 src/qml 前缀,
# 导致运行时找不到 :/qt/qml/com/hri/app/main.qml)
set_source_files_properties(src/qml/main.qml PROPERTIES QT_RESOURCE_ALIAS "main.qml")
set_source_files_properties(src/qml/pages/HomePage.qml PROPERTIES QT_RESOURCE_ALIAS "pages/HomePage.qml")
set_source_files_properties(src/qml/pages/StatusPage.qml PROPERTIES QT_RESOURCE_ALIAS "pages/StatusPage.qml")
set_source_files_properties(src/qml/pages/SettingsPage.qml PROPERTIES QT_RESOURCE_ALIAS "pages/SettingsPage.qml")
set_source_files_properties(src/qml/components/BottomNav.qml PROPERTIES QT_RESOURCE_ALIAS "components/BottomNav.qml")
set_source_files_properties(src/qml/components/TaskCard.qml PROPERTIES QT_RESOURCE_ALIAS "components/TaskCard.qml")
set_source_files_properties(src/qml/components/TaskChain.qml PROPERTIES QT_RESOURCE_ALIAS "components/TaskChain.qml")
set_source_files_properties(src/qml/components/SpeedControl.qml PROPERTIES QT_RESOURCE_ALIAS "components/SpeedControl.qml")
set_source_files_properties(src/qml/components/StartButton.qml PROPERTIES QT_RESOURCE_ALIAS "components/StartButton.qml")
set_source_files_properties(src/qml/components/TaskCockpit.qml PROPERTIES QT_RESOURCE_ALIAS "components/TaskCockpit.qml")
set_source_files_properties(src/qml/components/GlassPanel.qml PROPERTIES QT_RESOURCE_ALIAS "components/GlassPanel.qml")
set_source_files_properties(src/qml/components/GlyphIcon.qml PROPERTIES QT_RESOURCE_ALIAS "components/GlyphIcon.qml")
set_source_files_properties(src/qml/Theme/Theme.qml PROPERTIES QT_RESOURCE_ALIAS "Theme/Theme.qml" QT_QML_SINGLETON_TYPE TRUE)

# QML 模块: URI 为 com.hri.app; QT_RESOURCE_ALIAS 决定各文件的资源路径
qt_add_qml_module(hri_app
    URI com.hri.app
    VERSION 1.0
    QML_FILES
        src/qml/main.qml
        src/qml/pages/HomePage.qml
        src/qml/pages/StatusPage.qml
        src/qml/pages/SettingsPage.qml
        src/qml/components/BottomNav.qml
        src/qml/components/TaskCard.qml
        src/qml/components/TaskChain.qml
        src/qml/components/SpeedControl.qml
        src/qml/components/StartButton.qml
        src/qml/components/TaskCockpit.qml
        src/qml/components/GlassPanel.qml
        src/qml/components/GlyphIcon.qml
        src/qml/Theme/Theme.qml
    RESOURCE_PREFIX /qt/qml
)

target_link_libraries(hri_app PRIVATE
    Qt6::Core
    Qt6::Gui
    Qt6::Quick
    Qt6::Qml
    Qt6::Widgets
    Qt6::Network
    Qt6::QuickControls2
)

# 链接后把 QML 模块资源打进最终可执行文件
qt_finalize_executable(hri_app)
