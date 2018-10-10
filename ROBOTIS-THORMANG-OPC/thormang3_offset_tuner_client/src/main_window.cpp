/*******************************************************************************
 * Copyright (c) 2016, ROBOTIS CO., LTD.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of ROBOTIS nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

/* Author: Kayman Jung, sch */

/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/thormang3_offset_tuner_client/main_window.hpp"

/*****************************************************************************
 ** Namespaces
 *****************************************************************************/

namespace thormang3_offset_tuner_client
{

using namespace Qt;

/*****************************************************************************
 ** Implementation [MainWindow]
 *****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
    : QMainWindow(parent),
      offset_tuner_qnode_(argc, argv)
{
  ui_.setupUi(this);  // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
  QObject::connect(ui_.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt()));  // qApp is a global variable for the application

  setWindowIcon(QIcon(":/images/icon.png"));
  ui_.tab_manager->setCurrentIndex(0);  // ensure the first tab is showing - qt-designer should have this already hardwired, but often loses it (settings?).
  QObject::connect(&offset_tuner_qnode_, SIGNAL(rosShutdown()), this, SLOT(close()));

  all_torque_on_ = false;

  spinBox_list_.push_back("goal");
  spinBox_list_.push_back("offset");
  spinBox_list_.push_back("mod");
  spinBox_list_.push_back("present");
  spinBox_list_.push_back("p_gain");
  spinBox_list_.push_back("i_gain");
  spinBox_list_.push_back("d_gain");

  /****************************
   ** Connect
   ****************************/

  qRegisterMetaType<thormang3_offset_tuner_msgs::JointOffsetPositionData>(
      "thormang3_offset_tuner_msgs::JointOffsetPositionData");
  QObject::connect(&offset_tuner_qnode_,
                   SIGNAL(update_present_joint_offset_data(thormang3_offset_tuner_msgs::JointOffsetPositionData)), this,
                   SLOT(update_joint_offset_data_spinbox(thormang3_offset_tuner_msgs::JointOffsetPositionData)));

  /*********************
   ** Logging
   **********************/
  ui_.view_logging->setModel(offset_tuner_qnode_.loggingModel());
  QObject::connect(&offset_tuner_qnode_, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));

  /****************************
   ** Connect
   ****************************/

  /*********************
   ** Auto Start
   **********************/
  offset_tuner_qnode_.init();

  // make ui
  MakeUI();
}

MainWindow::~MainWindow()
{
}

/*****************************************************************************
 ** Implementation [Slots]
 *****************************************************************************/

void MainWindow::on_save_button_clicked(bool check)
{
  std_msgs::String msg;
  msg.data = "save";

  offset_tuner_qnode_.send_command_msg(msg);
}

void MainWindow::on_inipose_button_clicked(bool checck)
{
  std_msgs::String msg;
  msg.data = "ini_pose";

  offset_tuner_qnode_.send_command_msg(msg);
}

void MainWindow::on_refresh_button_clicked(bool check)
{
  offset_tuner_qnode_.getPresentJointOffsetData();
}

void MainWindow::all_torque_on_button_clicked(QObject *button_group)
{
  all_torque_on_ = true;

  QButtonGroup* torque_button_group = qobject_cast<QButtonGroup*>(button_group);
  if (!torque_button_group)  // this is just a safety check
    return;

  QList<QAbstractButton *> torque_buttons = torque_button_group->buttons();
  for (int ix = 0; ix < torque_buttons.size(); ix++)
  {
    if (torque_buttons[ix]->isChecked() == false)
      torque_buttons[ix]->click();
  }

  offset_tuner_qnode_.getPresentJointOffsetData();

  all_torque_on_ = false;
}

void MainWindow::all_torque_off_button_clicked(QObject *button_group)
{
  QButtonGroup* torque_button_group = qobject_cast<QButtonGroup*>(button_group);
  if (!torque_button_group)  // this is just a safety check
    return;

  QList<QAbstractButton *> torque_buttons = torque_button_group->buttons();
  for (int ix = 0; ix < torque_buttons.size(); ix++)
  {
    if (torque_buttons[ix]->isChecked() == true)
      torque_buttons[ix]->click();
  }
}

//void MainWindow::checkbox_clicked(QString joint_name)
void MainWindow::torque_checkbox_clicked(QWidget *widget)
{
  QCheckBox* checkBox = qobject_cast<QCheckBox*>(widget);
  if (!checkBox)  // this is just a safety check
    return;

  std::string joint_name = checkBox->text().toStdString();
  bool _is_on = checkBox->isChecked();

  QList<QAbstractSpinBox *> spinbox_list = joint_spinbox_map_[joint_name];

  for (int ix = 0; ix < spinbox_list.size(); ix++)
  {
    spinbox_list[ix]->setEnabled(_is_on);
  }

  publish_torque_msgs(joint_name, _is_on);

}

void MainWindow::publish_torque_msgs(std::string &joint_name, bool torque_on)
{
  thormang3_offset_tuner_msgs::JointTorqueOnOffArray msg_array;
  thormang3_offset_tuner_msgs::JointTorqueOnOff msg;

  msg.joint_name = joint_name;
  msg.torque_enable = torque_on;

  msg_array.torque_enable_data.push_back(msg);

  offset_tuner_qnode_.send_torque_enable_msg(msg_array);

  if (all_torque_on_ == false)
    offset_tuner_qnode_.getPresentJointOffsetData();
}

void MainWindow::spinBox_valueChanged(QString joint_name)
{
  if (offset_tuner_qnode_.is_refresh() == true)
    return;

  thormang3_offset_tuner_msgs::JointOffsetData msg;
  std::string current_joint_name = joint_name.toStdString();

  QList<QAbstractSpinBox *> spinbox_list = joint_spinbox_map_[current_joint_name];
  QDoubleSpinBox *mod_spinBox;

  msg.joint_name = current_joint_name;

  for (int ix = 0; ix < spinbox_list.size(); ix++)
  {
    if (spinbox_list[ix]->whatsThis().toStdString() == "goal")
    {
      QDoubleSpinBox* spinbox = qobject_cast<QDoubleSpinBox*>(spinbox_list[ix]);
      if (!spinbox)  // this is just a safety check
        continue;

      msg.goal_value = spinbox->value() * M_PI / 180.0;
    }
    else if (spinbox_list[ix]->whatsThis().toStdString() == "offset")
    {
      QDoubleSpinBox* spinbox = qobject_cast<QDoubleSpinBox*>(spinbox_list[ix]);
      if (!spinbox)  // this is just a safety check
        continue;

      msg.offset_value = spinbox->value() * M_PI / 180.0;
    }
    else if (spinbox_list[ix]->whatsThis().toStdString() == "mod")
    {
      mod_spinBox = qobject_cast<QDoubleSpinBox*>(spinbox_list[ix]);
    }
    else if (spinbox_list[ix]->whatsThis().toStdString() == "p_gain")
    {
      QSpinBox* spinbox = qobject_cast<QSpinBox*>(spinbox_list[ix]);
      if (!spinbox)  // this is just a safety check
        continue;

      msg.p_gain = spinbox->value();
    }
    else if (spinbox_list[ix]->whatsThis().toStdString() == "i_gain")
    {
      QSpinBox* spinbox = qobject_cast<QSpinBox*>(spinbox_list[ix]);
      if (!spinbox)  // this is just a safety check
        continue;

      msg.i_gain = spinbox->value();
    }
    else if (spinbox_list[ix]->whatsThis().toStdString() == "d_gain")
    {
      QSpinBox* spinbox = qobject_cast<QSpinBox*>(spinbox_list[ix]);
      if (!spinbox)  // this is just a safety check
        continue;

      msg.d_gain = spinbox->value();
    }
  }

  if (mod_spinBox)  // this is just a safety check
    mod_spinBox->setValue((msg.goal_value + msg.offset_value) * 180.0 / M_PI);

  offset_tuner_qnode_.send_joint_offset_data_msg(msg);
}

void MainWindow::update_joint_offset_data_spinbox(thormang3_offset_tuner_msgs::JointOffsetPositionData msg)
{
  std::string joint_name = msg.joint_name;

  QList<QAbstractSpinBox *> spinbox_list = joint_spinbox_map_[joint_name];

  for (int ix = 0; ix < spinbox_list.size(); ix++)
  {
    if (spinbox_list[ix]->whatsThis().toStdString() == "goal")
    {
      QDoubleSpinBox* spinbox = qobject_cast<QDoubleSpinBox*>(spinbox_list[ix]);
      if (!spinbox)  // this is just a safety check
        continue;

      spinbox->setValue(msg.goal_value * 180.0 / M_PI);
    }
    else if (spinbox_list[ix]->whatsThis().toStdString() == "offset")
    {
      QDoubleSpinBox* spinbox = qobject_cast<QDoubleSpinBox*>(spinbox_list[ix]);
      if (!spinbox)  // this is just a safety check
        continue;

      spinbox->setValue(msg.offset_value * 180.0 / M_PI);
    }
    else if (spinbox_list[ix]->whatsThis().toStdString() == "present")
    {
      QDoubleSpinBox* spinbox = qobject_cast<QDoubleSpinBox*>(spinbox_list[ix]);
      if (!spinbox)  // this is just a safety check
        continue;

      spinbox->setValue(msg.present_value * 180.0 / M_PI);
    }
    else if (spinbox_list[ix]->whatsThis().toStdString() == "p_gain")
    {
      QSpinBox* spinbox = qobject_cast<QSpinBox*>(spinbox_list[ix]);
      if (!spinbox)  // this is just a safety check
        continue;

      spinbox->setValue(msg.p_gain);
    }
    else if (spinbox_list[ix]->whatsThis().toStdString() == "i_gain")
    {
      QSpinBox* spinbox = qobject_cast<QSpinBox*>(spinbox_list[ix]);
      if (!spinbox)  // this is just a safety check
        continue;

      spinbox->setValue(msg.i_gain);
    }
    else if (spinbox_list[ix]->whatsThis().toStdString() == "d_gain")
    {
      QSpinBox* spinbox = qobject_cast<QSpinBox*>(spinbox_list[ix]);
      if (!spinbox)  // this is just a safety check
        continue;

      spinbox->setValue(msg.d_gain);
    }
  }
}

/*****************************************************************************
 ** Implemenation [Slots][manually connected]
 *****************************************************************************/

/**
 * This function is signalled by the underlying model. When the model changes,
 * this will drop the cursor down to the last line in the QListview to ensure
 * the user can always see the latest log message.
 */
void MainWindow::updateLoggingView()
{
  ui_.view_logging->scrollToBottom();
}

void MainWindow::MakeUI()
{
  MakeTabUI(ui_.right_arm_group, ui_.right_arm_torque, right_arm_button_group_,
            offset_tuner_qnode_.right_arm_offset_group);
  MakeTabUI(ui_.left_arm_group, ui_.left_arm_torque, left_arm_button_group_, offset_tuner_qnode_.left_arm_offset_group);
  MakeTabUI(ui_.leg_group, ui_.leg_torque, legs_button_group_, offset_tuner_qnode_.legs_offset_group);
  MakeTabUI(ui_.body_group, ui_.body_torque, body_button_group_, offset_tuner_qnode_.body_offset_group);
}

void MainWindow::MakeTabUI(QGroupBox *joint_widget, QGroupBox *torque_widget, QButtonGroup *button_group,
                           std::map<int, std::string> &offset_group)
{
  QSignalMapper *torque_checkbox_signalMapper = new QSignalMapper(this);

  QGridLayout *grid_layout = (QGridLayout *) joint_widget->layout();
  QGridLayout *torque_layout = (QGridLayout *) torque_widget->layout();

  button_group = new QButtonGroup();
  button_group->setExclusive(false);

  int row = 3;
  int torque_checkbox_index = 0;
  int torque_row = 1;
  int torque_col = 0;
  for (std::map<int, std::string>::iterator map_iter = offset_group.begin(); map_iter != offset_group.end(); ++map_iter)
  {
    QSignalMapper *spingox_signalMapper = new QSignalMapper(this);
    QList<QAbstractSpinBox *> spinbox_list;

    // spin_box
    int col = 0;
    int size = 1;
    std::string joint_name = map_iter->second;
    QString q_joint_name = QString::fromStdString(joint_name);

    // label
    QLabel *joint_label = new QLabel(q_joint_name);
    grid_layout->addWidget(joint_label, row, col++, 1, size);

    // double spin box
    for (int ix = 0; ix < 4; ix++)
    {
      QDoubleSpinBox *spin_box = new QDoubleSpinBox();
      spin_box->setWhatsThis(tr(spinBox_list_[ix].c_str()));
      spin_box->setMinimum(-360);
      spin_box->setMaximum(360);
      spin_box->setSingleStep(0.05);

      switch (ix)
      {
        case 2:
        case 3:
          spin_box->setReadOnly(true);
          break;

        default:
          spingox_signalMapper->setMapping(spin_box, q_joint_name);
          QObject::connect(spin_box, SIGNAL(valueChanged(QString)), spingox_signalMapper, SLOT(map()));
          break;
      }

      grid_layout->addWidget(spin_box, row, col++, 1, size);

      spinbox_list.append(spin_box);
    }

    // spin box
    for (int ix = 0; ix < 3; ix++)
    {
      QSpinBox *spin_box = new QSpinBox();
      spin_box->setWhatsThis(tr(spinBox_list_[ix + 4].c_str()));
      spin_box->setMinimum(0);
      spin_box->setMaximum(1000);
      spin_box->setSingleStep(1);

      switch (ix)
      {
        case 0:
          spin_box->setValue(32);

          spingox_signalMapper->setMapping(spin_box, q_joint_name);
          QObject::connect(spin_box, SIGNAL(valueChanged(QString)), spingox_signalMapper, SLOT(map()));
          break;

        default:
          spin_box->setReadOnly(true);
          break;
      }

      grid_layout->addWidget(spin_box, row, col++, 1, size);

      spinbox_list.append(spin_box);
    }

    // spinbox
    joint_spinbox_map_[joint_name] = spinbox_list;
    QObject::connect(spingox_signalMapper, SIGNAL(mapped(QString)), this, SLOT(spinBox_valueChanged(QString)));

    row += 1;

    // torque checkbox
    torque_row = torque_checkbox_index / 6;
    torque_col = torque_checkbox_index % 6;

    QCheckBox *check_box = new QCheckBox(q_joint_name);
    check_box->setChecked(true);
    torque_layout->addWidget(check_box, torque_row, torque_col, 1, size);
    button_group->addButton(check_box);

    torque_checkbox_signalMapper->setMapping(check_box, check_box);
    QObject::connect(check_box, SIGNAL(clicked()), torque_checkbox_signalMapper, SLOT(map()));

    torque_checkbox_index += 1;
  }

  // all torque on
  QSignalMapper *torque_on_signalMapper = new QSignalMapper(this);
  QPushButton *on_button = new QPushButton(tr("All torque ON"));
  torque_layout->addWidget(on_button, torque_row + 1, 4, 1, 1);
  torque_on_signalMapper->setMapping(on_button, button_group);
  QObject::connect(on_button, SIGNAL(clicked()), torque_on_signalMapper, SLOT(map()));
  QObject::connect(torque_on_signalMapper, SIGNAL(mapped(QObject*)), this,
                   SLOT(all_torque_on_button_clicked(QObject*)));

  // all torque off
  QSignalMapper *torque_off_signalMapper = new QSignalMapper(this);
  QPushButton *off_button = new QPushButton(tr("All torque OFF"));
  torque_layout->addWidget(off_button, torque_row + 1, 5, 1, 1);
  torque_off_signalMapper->setMapping(off_button, button_group);
  QObject::connect(off_button, SIGNAL(clicked()), torque_off_signalMapper, SLOT(map()));
  QObject::connect(torque_off_signalMapper, SIGNAL(mapped(QObject*)), this,
                   SLOT(all_torque_off_button_clicked(QObject*)));

  QObject::connect(torque_checkbox_signalMapper, SIGNAL(mapped(QWidget*)), this,
                   SLOT(torque_checkbox_clicked(QWidget*)));
}

/*****************************************************************************
 ** Implementation [Menu]
 *****************************************************************************/

void MainWindow::on_actionAbout_triggered()
{
  QMessageBox::about(this, tr("About ..."), tr("<h2>THORMANG3 Offset tuner client</h2><p>Copyright ROBOTIS</p>"));
}

/*****************************************************************************
 ** Implementation [Configuration]
 *****************************************************************************/

void MainWindow::closeEvent(QCloseEvent *event)
{
  QMainWindow::closeEvent(event);
}

}  // namespace thormang3_offset_tuner_client

