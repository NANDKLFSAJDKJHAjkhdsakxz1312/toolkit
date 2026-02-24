#include <QApplication>
#include <QFontDatabase>
#include <QScreen>
#include <sys/mman.h>
#include "mainwindow.h"

void lockMemory()
{
    // 锁定当前和未来分配的所有内存
    if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1)
    {
        printf("锁定内存失败！需要 sudo 权限。\n");
    }
}
int main(int argc, char *argv[])
{
  lockMemory();
  qputenv("QT_QPA_FONTDIR", "/usr/share/fonts/truetype");
  qputenv("XDG_RUNTIME_DIR", "/tmp/runtime-root");

  // 1. 启用自动DPI缩放
  QApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
  QApplication::setAttribute(Qt::AA_UseHighDpiPixmaps);

  // 2. 设置环境变量
  qputenv("QT_AUTO_SCREEN_SCALE_FACTOR", "1");

  QApplication a(argc, argv);

  // 强制加载字体
  QFontDatabase::addApplicationFont("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf");

  // 强制设置字体
  QFont font("DejaVu Sans", 8);
  a.setFont(font);

  // 全局样式
  a.setStyleSheet("font-size: 8px; min-height: 12px;");

  MainWindow w;
  w.show();
  return a.exec();
}
