#ifndef ejemplo1_H
#define ejemplo1_H

#include <QtGui>
#include "ui_counterDlg.h"
#include "QTimer"
#include "timer.h"

class ejemplo1 : public QWidget, public Ui_Counter
{
    Q_OBJECT
    public:
        ejemplo1();

    public slots:
        void doButton();
        void modifyFreq();
        void updateDisplay(int i);
        void doReset();

private:
    bool m_running{false};
    int m_counter{0};
    QTimer *m_timer{};
    Timer *o_timer{};
};

#endif // ejemplo1_H
