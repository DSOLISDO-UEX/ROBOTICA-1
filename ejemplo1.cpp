#include "ejemplo1.h"

ejemplo1::ejemplo1() : Ui_Counter() {
    setupUi(this);
    show();

    m_timer = new QTimer(this);
    button->setText("Start");

    connect(m_timer, &QTimer::timeout, this, &ejemplo1::updateDisplay);
    connect(button, SIGNAL(clicked()), this, SLOT(doButton()));
    connect(freqSlider, SIGNAL(valueChanged(int)), this, SLOT(modifyFreq()));
    connect(reset, SIGNAL(clicked()), this, SLOT(doReset()));
}

void ejemplo1::updateDisplay() {
    m_counter++;
    lcdNumber->display(m_counter);
}

void ejemplo1::modifyFreq() {
    m_timer->setInterval(freqSlider->value());
}

void ejemplo1::doReset() {
    m_counter = 0;
    lcdNumber->display(m_counter);
}

void ejemplo1::doButton() {
    if (m_running) {
        button->setText("Start");
        m_timer->stop();
        m_running = false;
    } else {
        button->setText("Stop");
        m_timer->stop();
        m_timer->start(freqSlider->value());
        m_running = true;
    }
}
