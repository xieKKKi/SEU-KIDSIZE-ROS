#include <QApplication>
#include <fstream>
#include "action_debuger.hpp"
#include <seumath/math.hpp>

#define SCALE_K  0.001f
#define DEG_RANGE 180.0f
#define MAX_POS_ID 100

using namespace std;
using namespace robot;
using namespace seumath;
using namespace Eigen;

ActionDebuger::ActionDebuger(ros::NodeHandle &n): node(n)
{
    initStatusBar();
    std::string robot_file, offset_file;
    try{
        ros::param::get("robot_file", robot_file);
        ros::param::get("offset_file", offset_file);
        ros::param::get("action_file", act_file_);
    }
    catch(ros::InvalidNameException &e){
        ROS_ERROR("%s", e.what());
        exit(0);
    }
    
    mRbt = make_shared<Robot>(robot_file, offset_file);
    action_eng_ = make_shared<ActionEngine>(act_file_, mRbt);

    robot_gl_ = new RobotGL(mRbt->get_main_bone(), mRbt->get_joint_map());
    QVBoxLayout *leftLayout = new QVBoxLayout;
    leftLayout->addWidget(robot_gl_);

    mKsliders.clear();
    CKSlider *slider;

    slider = new CKSlider("X");
    connect(slider, &CKSlider::valueChanged, this, &ActionDebuger::procX);
    mKsliders.push_back(slider);
    slider = new CKSlider("Y");
    connect(slider, &CKSlider::valueChanged, this, &ActionDebuger::procY);
    mKsliders.push_back(slider);
    slider = new CKSlider("Z");
    connect(slider, &CKSlider::valueChanged, this, &ActionDebuger::procZ);
    mKsliders.push_back(slider);
    slider = new CKSlider("Roll");
    connect(slider, &CKSlider::valueChanged, this, &ActionDebuger::procRoll);
    mKsliders.push_back(slider);
    slider = new CKSlider("Pitch");
    connect(slider, &CKSlider::valueChanged, this, &ActionDebuger::procPitch);
    mKsliders.push_back(slider);
    slider = new CKSlider("Yaw");
    connect(slider, &CKSlider::valueChanged, this, &ActionDebuger::procYaw);
    mKsliders.push_back(slider);

    mSliderGroup = new QGroupBox();
    QVBoxLayout *sliderLayout = new QVBoxLayout();

    for (auto &s : mKsliders)
    {
        sliderLayout->addWidget(s);
    }

    mSliderGroup->setLayout(sliderLayout);

    mButtonSavePos = new QPushButton(tr("Save Pos"));
    head = new QRadioButton("Head");
    body = new QRadioButton("Body");
    leftArm = new QRadioButton("Left Arm");
    rightArm = new QRadioButton("Right Arm");
    leftFoot = new QRadioButton("Left Foot");
    rightFoot = new QRadioButton("Right Foot");

    m_pJDListWidget1 = new QListWidget;
    m_pJDListWidget2 = new QListWidget;

    motionBtnGroup = new QButtonGroup();
    motionBtnGroup->addButton(head, static_cast<int>(MOTION_HEAD));
    motionBtnGroup->addButton(body, static_cast<int>(MOTION_BODY));
    motionBtnGroup->addButton(leftArm, static_cast<int>(MOTION_LEFT_HAND));
    motionBtnGroup->addButton(rightArm, static_cast<int>(MOTION_RIGHT_HAND));
    motionBtnGroup->addButton(leftFoot, static_cast<int>(MOTION_LEFT_FOOT));
    motionBtnGroup->addButton(rightFoot, static_cast<int>(MOTION_RIGHT_FOOT));
    body->setChecked(true);

    QHBoxLayout *hbLayout = new QHBoxLayout();
    hbLayout->addWidget(head);
    hbLayout->addWidget(body);
    QHBoxLayout *armLayout = new QHBoxLayout();
    armLayout->addWidget(rightArm);
    armLayout->addWidget(leftArm);
    QHBoxLayout *footLayout = new QHBoxLayout();
    footLayout->addWidget(rightFoot);
    footLayout->addWidget(leftFoot);
    QHBoxLayout *jdLayout = new QHBoxLayout();
    jdLayout->addWidget(m_pJDListWidget1);
    jdLayout->addWidget(m_pJDListWidget2);

    QVBoxLayout *midLayout = new QVBoxLayout;
    midLayout->setAlignment(Qt::AlignCenter);
    midLayout->addWidget(mSliderGroup);
    midLayout->addLayout(hbLayout);
    midLayout->addLayout(armLayout);
    midLayout->addLayout(footLayout);
    midLayout->addLayout(jdLayout);
    midLayout->addWidget(mButtonSavePos);

    m_pPosListWidget = new QListWidget();
    m_pPosListWidget->setMinimumWidth(240);
    btnrunPos = new QPushButton("Run Pos");
    btnOutWebots = new QPushButton("Export Webots");
    QVBoxLayout *posLayout = new QVBoxLayout;
    posLayout->addWidget(m_pPosListWidget);
    posLayout->addWidget(btnrunPos);
    posLayout->addWidget(btnOutWebots);

    m_pActListWidget = new QListWidget();
    mButtonInsertPosFront = new QPushButton(tr("Insert Pos Front"));
    mButtonInsertPosBack = new QPushButton(tr("Insert Pos Back"));
    mButtonDeletePos = new QPushButton(tr("Delete Pos"));
    mButtonSaveAction = new QPushButton(tr("Save Actions"));
    mButtonAddAction = new QPushButton(tr("Add Action"));
    mButtonDeleteAction = new QPushButton(tr("Delete Action"));

    QVBoxLayout *bactLayout = new QVBoxLayout;
    bactLayout->addWidget(mButtonInsertPosFront);
    bactLayout->addWidget(mButtonInsertPosBack);
    bactLayout->addWidget(mButtonDeletePos);
    bactLayout->addWidget(mButtonSaveAction);
    bactLayout->addWidget(mButtonAddAction);
    bactLayout->addWidget(mButtonDeleteAction);
    bactLayout->addWidget(m_pActListWidget);

    QHBoxLayout *mainLayout = new QHBoxLayout;
    mainLayout->addLayout(leftLayout);
    mainLayout->addLayout(midLayout);
    mainLayout->addLayout(posLayout);
    mainLayout->addLayout(bactLayout);

    QWidget *mainWidget  = new QWidget();
    mainWidget->setLayout(mainLayout);
    this->setCentralWidget(mainWidget);

    pos_saved = true;
    last_pos_id = MAX_POS_ID;
    initActs();
    initPoseMap();
    initJDInfo();

    timer = new QTimer;
    timer->start(1000);

    connect(m_pActListWidget, &QListWidget::itemClicked, this, &ActionDebuger::procActSelect);
    connect(m_pPosListWidget, &QListWidget::itemClicked, this, &ActionDebuger::procPosSelect);
    connect(mButtonAddAction, &QPushButton::clicked, this, &ActionDebuger::procButtonAddAction);
    connect(mButtonDeleteAction, &QPushButton::clicked, this, &ActionDebuger::procButtonDeleteAction);
    connect(mButtonSaveAction, &QPushButton::clicked, this, &ActionDebuger::procButtonSaveAction);
    connect(mButtonInsertPosFront, &QPushButton::clicked, this, &ActionDebuger::procButtonInsertPosFront);
    connect(mButtonInsertPosBack, &QPushButton::clicked, this, &ActionDebuger::procButtonInsertPosBack);
    connect(mButtonDeletePos, &QPushButton::clicked, this, &ActionDebuger::procButtonDeletePos);
    connect(mButtonSavePos, &QPushButton::clicked, this, &ActionDebuger::procButtonSavePos);
    connect(btnrunPos, &QPushButton::clicked, this, &ActionDebuger::procButtonRunPos);
    connect(btnOutWebots, &QPushButton::clicked, this, &ActionDebuger::procButtonWebots);
    connect(motionBtnGroup, static_cast<void(QButtonGroup::*)(int)>(&QButtonGroup::buttonClicked), this, &ActionDebuger::updateSlider);
    connect(timer, &QTimer::timeout, this, &ActionDebuger::procTimer);
}

void ActionDebuger::procPosNameChanged(int id)
{
    CPosListWidget *pCur_PosWidget = (CPosListWidget *) m_pPosListWidget->itemWidget(m_pPosListWidget->item(id - 1));

    if (pCur_PosWidget == nullptr)
    {
        cout << "empty\n";
    }

    string new_name = pCur_PosWidget->pos_name->text().toStdString();
    string old_name = pCur_PosWidget->pos_name_;
    string act_name = m_pActListWidget->currentItem()->text().toStdString();
    auto &pos_map_ = action_eng_->get_pos_map();
    auto &act_map_ = action_eng_->get_act_map();
    if (pos_map_.find(new_name) == pos_map_.end())
    {
        QMessageBox::StandardButton reply = QMessageBox::question(this, "Warning", "pos: " + QString::fromStdString(new_name) + " does not exist, create it?",
                                            QMessageBox::Yes | QMessageBox::No, QMessageBox::Yes);

        if (reply != QMessageBox::StandardButton::Yes)
        {
            pCur_PosWidget->pos_name->setText(QString::fromStdString(old_name));
            return;
        }

        pCur_PosWidget->pos_name->setText(QString::fromStdString(new_name));
        RobotPos tempPos;
        tempPos.name = new_name;
        tempPos.pose_info = pos_map_[old_name].pose_info;
        pos_map_[new_name] = tempPos;
    }

    act_map_[act_name].poses[id - 1].pos_name = new_name;
}

void ActionDebuger::procPosTimeChanged(int id)
{
    CPosListWidget *pCur_PosWidget = (CPosListWidget *) m_pPosListWidget->itemWidget(m_pPosListWidget->item(id - 1));
    auto &act_map_ = action_eng_->get_act_map();
    string act_name = m_pActListWidget->currentItem()->text().toStdString();
    act_map_[act_name].poses[id - 1].act_time = pCur_PosWidget->pos_time->text().toInt();
}

void ActionDebuger::procTimer()
{

}

void ActionDebuger::initActs()
{
    m_pPosListWidget->clear();
    m_pActListWidget->clear();
    auto &act_map_ = action_eng_->get_act_map();
    for (auto &act : act_map_)
    {
        m_pActListWidget->addItem(QString::fromStdString(act.second.name));
    }

    mSliderGroup->setEnabled(false);
}

void ActionDebuger::initPoseMap()
{
    motion_ = MOTION_BODY;
    last_motion = MOTION_BODY;
    RobotPose temp = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    for (int i = 1; i <= 6; i++)
    {
        pose_map_[static_cast<RobotMotion >(i)] = temp;
    }

    for (auto &j : mRbt->get_joint_map())
    {
        joint_degs_[j.second->jid] = j.second->current_deg;
    }
}

void ActionDebuger::initStatusBar()
{
    QStatusBar *bar = statusBar();
    motionlab = new QLabel;
    valuelab = new QLabel;
    curractlab = new QLabel;
    currposlab = new QLabel;
    netstatuslab = new QLabel;
    motionlab->setMinimumWidth(60);
    valuelab->setMinimumWidth(60);
    curractlab->setMinimumWidth(60);
    currposlab->setMinimumWidth(60);
    netstatuslab->setMinimumWidth(100);
    bar->addWidget(motionlab);
    bar->addWidget(valuelab);
    bar->addWidget(curractlab);
    bar->addWidget(currposlab);
    bar->addWidget(netstatuslab);
}

void ActionDebuger::initJDInfo()
{
    mJDInfos.clear();
    CJointDegWidget *pJDWidget;

    QListWidgetItem *pListItem;;
    m_pJDListWidget1->clear();
    m_pJDListWidget2->clear();

    for (auto &jd : joint_degs_)
    {
        pListItem = new QListWidgetItem;
        pJDWidget = new CJointDegWidget(mRbt->get_joint(jd.first)->name, jd.second);
        pJDWidget->show();
        mJDInfos[mRbt->get_joint(jd.first)->name] = pJDWidget;

        if (mRbt->get_joint(jd.first)->name == "jhead2" || mRbt->get_joint(jd.first)->name.find("jr") != string::npos)
        {
            m_pJDListWidget1->addItem(pListItem);
            m_pJDListWidget1->setItemWidget(pListItem, pJDWidget);
        }
        else
        {
            m_pJDListWidget2->addItem(pListItem);
            m_pJDListWidget2->setItemWidget(pListItem, pJDWidget);
        }

        pListItem->setSizeHint(QSize(pJDWidget->rect().width(), pJDWidget->rect().height()));
    }
}

void ActionDebuger::updateSlider(int id)
{
    motion_ = static_cast<RobotMotion >(id);
    motionlab->setText(QString::fromStdString(get_name_by_motion(motion_)));

    std::vector<CKSlider *>::iterator iter = mKsliders.begin();
    (*iter)->slider->setValue(pose_map_[motion_].x / SCALE_K);
    (*iter)->nameLab->setText("X");

    if (motion_ == MOTION_HEAD)
    {
        (*iter)->nameLab->setText("Head Pitch");
    }

    iter++;
    (*iter)->slider->setValue(pose_map_[motion_].y / SCALE_K);
    (*iter)->nameLab->setText("Y");
    (*iter)->slider->setEnabled(true);

    if (motion_ == MOTION_LEFT_HAND || motion_ == MOTION_RIGHT_HAND)
    {
        (*iter)->slider->setEnabled(false);
    }
    else if (motion_ == MOTION_HEAD)
    {
        (*iter)->nameLab->setText("Head Yaw");
    }

    iter++;
    (*iter)->slider->setValue(pose_map_[motion_].z / SCALE_K);
    (*iter)->nameLab->setText("Z");
    (*iter)->slider->setEnabled(true);

    if (motion_ == MOTION_HEAD)
    {
        (*iter)->slider->setEnabled(false);
    }

    iter++;
    (*iter)->slider->setValue(pose_map_[motion_].roll);
    (*iter)->slider->setEnabled(true);

    if (motion_ == MOTION_LEFT_HAND || motion_ == MOTION_RIGHT_HAND || motion_ == MOTION_HEAD)
    {
        (*iter)->slider->setEnabled(false);
    }

    iter++;
    (*iter)->slider->setValue(pose_map_[motion_].pitch);
    (*iter)->slider->setEnabled(true);

    if (motion_ == MOTION_LEFT_HAND || motion_ == MOTION_RIGHT_HAND || motion_ == MOTION_HEAD)
    {
        (*iter)->slider->setEnabled(false);
    }

    iter++;
    (*iter)->slider->setValue(pose_map_[motion_].yaw);
    (*iter)->slider->setEnabled(true);

    if (motion_ == MOTION_LEFT_HAND || motion_ == MOTION_RIGHT_HAND || motion_ == MOTION_HEAD)
    {
        (*iter)->slider->setEnabled(false);
    }

    last_motion = motion_;
}

float ActionDebuger::get_deg_from_pose(const float &ps)
{
    int value = (int)(ps / SCALE_K);
    return DEG_RANGE / SLIDER_RANGE * value;
}

bool ActionDebuger::turn_joint()
{
    joint_degs_[mRbt->get_joint("jhead2")->jid] = get_deg_from_pose(pose_map_[MOTION_HEAD].x);
    joint_degs_[mRbt->get_joint("jhead1")->jid] = get_deg_from_pose(pose_map_[MOTION_HEAD].y);

    Vector3d lefthand, righthand;
    righthand[0] = pose_map_[MOTION_RIGHT_HAND].x;
    righthand[2] = pose_map_[MOTION_RIGHT_HAND].z;
    lefthand[0] = pose_map_[MOTION_LEFT_HAND].x;
    lefthand[2] = pose_map_[MOTION_LEFT_HAND].z;

    TransformMatrix body_mat, leftfoot_mat, rightfoot_mat;
    body_mat = mRbt->get_body_mat_from_pose(pose_map_[MOTION_BODY]);
    leftfoot_mat = mRbt->get_foot_mat_from_pose(pose_map_[MOTION_LEFT_FOOT], true);
    rightfoot_mat = mRbt->get_foot_mat_from_pose(pose_map_[MOTION_RIGHT_FOOT], false);

    vector<double> degs;

    if (mRbt->leg_inverse_kinematics(body_mat, leftfoot_mat, degs, true))
    {
        joint_degs_[mRbt->get_joint("jlhip3")->jid] = rad2deg(degs[0]);
        joint_degs_[mRbt->get_joint("jlhip2")->jid] = rad2deg(degs[1]);
        joint_degs_[mRbt->get_joint("jlhip1")->jid] = rad2deg(degs[2]);
        joint_degs_[mRbt->get_joint("jlknee")->jid] = rad2deg(degs[3]);
        joint_degs_[mRbt->get_joint("jlankle2")->jid] = rad2deg(degs[4]);
        joint_degs_[mRbt->get_joint("jlankle1")->jid] = rad2deg(degs[5]);
    }
    else
    {
        return false;
    }

    if (mRbt->leg_inverse_kinematics(body_mat, rightfoot_mat, degs, false))
    {
        joint_degs_[mRbt->get_joint("jrhip3")->jid] = rad2deg(degs[0]);
        joint_degs_[mRbt->get_joint("jrhip2")->jid] = rad2deg(degs[1]);
        joint_degs_[mRbt->get_joint("jrhip1")->jid] = rad2deg(degs[2]);
        joint_degs_[mRbt->get_joint("jrknee")->jid] = rad2deg(degs[3]);
        joint_degs_[mRbt->get_joint("jrankle2")->jid] = rad2deg(degs[4]);
        joint_degs_[mRbt->get_joint("jrankle1")->jid] = rad2deg(degs[5]);
    }
    else
    {
        return false;
    }

    if (mRbt->arm_inverse_kinematics(lefthand, degs))
    {
        joint_degs_[mRbt->get_joint("jlshoulder1")->jid] = rad2deg(degs[0]);
        joint_degs_[mRbt->get_joint("jlelbow")->jid] = -rad2deg(degs[1]);
    }
    else
    {
        return false;
    }

    if (mRbt->arm_inverse_kinematics(righthand, degs))
    {
        joint_degs_[mRbt->get_joint("jrshoulder1")->jid] = rad2deg(degs[0]);
        joint_degs_[mRbt->get_joint("jrelbow")->jid] = rad2deg(degs[1]);
    }
    else
    {
        return false;
    }
    robot_gl_->turn_joint(joint_degs_);
    updateJDInfo();
    return true;
}

void ActionDebuger::procX(int value)
{
    if (m_pPosListWidget->currentRow() == last_pos_id && motion_ == last_motion)
    {
        pos_saved = false;
    }

    float old_value = pose_map_[motion_].x;
    float v = ((float) value) * SCALE_K;
    valuelab->setText(QString::number(v, 'f'));
    pose_map_[motion_].x = v;

    if (leftArm->isChecked())
    {
        pose_map_[MOTION_LEFT_HAND].x = v;
    }

    if (rightArm->isChecked())
    {
        pose_map_[MOTION_RIGHT_HAND].x = v;
    }

    if (leftFoot->isChecked())
    {
        pose_map_[MOTION_LEFT_FOOT].x = v;
    }

    if (rightFoot->isChecked())
    {
        pose_map_[MOTION_RIGHT_FOOT].x = v;
    }

    if (!turn_joint())
    {
        pose_map_[motion_].x = old_value;
    }
}

void ActionDebuger::procY(int value)
{
    if (m_pPosListWidget->currentRow() == last_pos_id && motion_ == last_motion)
    {
        pos_saved = false;
    }

    float old_value = pose_map_[motion_].y;
    float v = ((float) value) * SCALE_K;
    valuelab->setText(QString::number(v, 'f'));
    pose_map_[motion_].y = v;

    if (leftFoot->isChecked())
    {
        pose_map_[MOTION_LEFT_FOOT].y = v;
    }

    if (rightFoot->isChecked())
    {
        pose_map_[MOTION_RIGHT_FOOT].y = v;
    }

    if (!turn_joint())
    {
        pose_map_[motion_].y = old_value;
    }
}

void ActionDebuger::procZ(int value)
{
    if (m_pPosListWidget->currentRow() == last_pos_id && motion_ == last_motion)
    {
        pos_saved = false;
    }

    float old_value = pose_map_[motion_].z;
    float v = ((float) value) * SCALE_K;
    valuelab->setText(QString::number(v, 'f'));
    pose_map_[motion_].z = v;

    if (leftArm->isChecked())
    {
        pose_map_[MOTION_LEFT_HAND].z = v;
    }

    if (rightArm->isChecked())
    {
        pose_map_[MOTION_RIGHT_HAND].z = v;
    }

    if (leftFoot->isChecked())
    {
        pose_map_[MOTION_LEFT_FOOT].z = v;
    }

    if (rightFoot->isChecked())
    {
        pose_map_[MOTION_RIGHT_FOOT].z = v;
    }

    if (!turn_joint())
    {
        pose_map_[motion_].z = old_value;
    }
}

void ActionDebuger::procRoll(int value)
{
    if (m_pPosListWidget->currentRow() == last_pos_id && motion_ == last_motion)
    {
        pos_saved = false;
    }

    float old_value = pose_map_[motion_].roll;
    valuelab->setText(QString::number(((float) value), 'f'));
    pose_map_[motion_].roll = ((float) value);

    if (leftFoot->isChecked())
    {
        pose_map_[MOTION_LEFT_FOOT].roll = ((float) value);
    }

    if (rightFoot->isChecked())
    {
        pose_map_[MOTION_RIGHT_FOOT].roll = ((float) value);
    }

    if (!turn_joint())
    {
        pose_map_[motion_].roll = old_value;
    }
}

void ActionDebuger::procPitch(int value)
{
    if (m_pPosListWidget->currentRow() == last_pos_id && motion_ == last_motion)
    {
        pos_saved = false;
    }

    float old_value = pose_map_[motion_].pitch;
    valuelab->setText(QString::number(((float) value), 'f'));
    pose_map_[motion_].pitch = ((float) value);

    if (leftFoot->isChecked())
    {
        pose_map_[MOTION_LEFT_FOOT].pitch = ((float) value);
    }

    if (rightFoot->isChecked())
    {
        pose_map_[MOTION_RIGHT_FOOT].pitch = ((float) value);
    }

    if (!turn_joint())
    {
        pose_map_[motion_].pitch = old_value;
    }
}

void ActionDebuger::procYaw(int value)
{
    if (m_pPosListWidget->currentRow() == last_pos_id && motion_ == last_motion)
    {
        pos_saved = false;
    }

    float old_value = pose_map_[motion_].yaw;
    valuelab->setText(QString::number(((float) value), 'f'));
    pose_map_[motion_].yaw = ((float) value);

    if (leftFoot->isChecked())
    {
        pose_map_[MOTION_LEFT_FOOT].yaw = ((float) value);
    }

    if (rightFoot->isChecked())
    {
        pose_map_[MOTION_RIGHT_FOOT].yaw = ((float) value);
    }

    if (!turn_joint())
    {
        pose_map_[motion_].yaw = old_value;
    }
}

void ActionDebuger::updateJDInfo()
{
    for (auto &jd : joint_degs_)
    {
        mJDInfos[mRbt->get_joint(jd.first)->name]->deg->setText(QString::number(jd.second, 'f', 2));
    }

    update();
}

void ActionDebuger::updatePosList(string act_name)
{
    auto &act_map_ = action_eng_->get_act_map();
    RobotAct act = act_map_[act_name];
    QListWidgetItem *pListItem;
    CPosListWidget *pPosWidget;
    int id = 0;
    m_pPosListWidget->clear();

    for (auto &pos : act.poses)
    {
        id++;
        pListItem = new QListWidgetItem;
        pPosWidget = new CPosListWidget(id, pos.pos_name, pos.act_time);
        pPosWidget->show();
        connect(pPosWidget, &CPosListWidget::nameChanged, this, &ActionDebuger::procPosNameChanged);
        connect(pPosWidget, &CPosListWidget::timeChanged, this, &ActionDebuger::procPosTimeChanged);
        m_pPosListWidget->addItem(pListItem);
        m_pPosListWidget->setItemWidget(pListItem, pPosWidget);
        pListItem->setSizeHint(QSize(pPosWidget->rect().width(), pPosWidget->rect().height()));
    }
}

void ActionDebuger::procActSelect(QListWidgetItem *item)
{
    if (!pos_saved)
    {
        QMessageBox::StandardButton reply = QMessageBox::question(this, "Warning", "pos have not been saved, continue?",
                                            QMessageBox::Yes | QMessageBox::No, QMessageBox::Yes);

        if (reply != QMessageBox::StandardButton::Yes)
        {
            m_pActListWidget->setCurrentRow(last_act_id);
            return;
        }
    }

    pos_saved = true;
    last_pos_id = MAX_POS_ID;
    last_act_id = m_pActListWidget->currentRow();
    curractlab->setText(item->text());
    currposlab->setText("");
    string name = m_pActListWidget->currentItem()->text().toStdString();
    updatePosList(name);
    mSliderGroup->setEnabled(false);
}

void ActionDebuger::procPosSelect(QListWidgetItem *item)
{
    if (!pos_saved)
    {
        QMessageBox::StandardButton reply = QMessageBox::question(this, "Warning", "pos have not been saved, continue?",
                                            QMessageBox::Yes | QMessageBox::No, QMessageBox::Yes);

        if (reply != QMessageBox::StandardButton::Yes)
        {
            m_pPosListWidget->setCurrentRow(last_pos_id);
            return;
        }
    }
    auto &pos_map_ = action_eng_->get_pos_map();
    pos_saved = true;
    CPosListWidget *pPosWidget = (CPosListWidget *) m_pPosListWidget->itemWidget(item);
    string pos_name = pPosWidget->pos_name_;
    currposlab->setText(QString::fromStdString(pos_name));
    valuelab->setText("");
    pose_map_ = pos_map_[pos_name].pose_info;
    updateSlider(static_cast<int>(motion_));
    turn_joint();
    mSliderGroup->setEnabled(true);
    last_pos_id = m_pPosListWidget->currentRow();
}

void ActionDebuger::procButtonInsertPosFront()
{
    if (m_pActListWidget->currentItem() == nullptr)
    {
        QMessageBox::warning(this, "Error", "No act select!");
        return;
    }

    string act_name = m_pActListWidget->currentItem()->text().toStdString();

    if (m_pPosListWidget->currentItem() == nullptr)
    {
        QMessageBox::warning(this, "Error", "No pos select!");
        return;
    }

    bool ok;
    string new_pos_name = QInputDialog::getText(this, tr("act name"), tr("input act name:"),
                          QLineEdit::Normal, nullptr, &ok).toStdString();

    if (!ok || new_pos_name.empty())
    {
        return;
    }

    bool exist = false;
    auto &pos_map_ = action_eng_->get_pos_map();
    auto &act_map_ = action_eng_->get_act_map();
    if (pos_map_.find(new_pos_name) != pos_map_.end())
    {
        exist = true;
        QMessageBox::StandardButton reply = QMessageBox::question(this, "Warning", "pos: " + QString::fromStdString(new_pos_name) + "  exists, use it?",
                                            QMessageBox::Yes | QMessageBox::No, QMessageBox::Yes);

        if (reply != QMessageBox::StandardButton::Yes)
        {
            return;
        }
    }

    RobotPos pos;
    RobotOnePos one_pos;
    one_pos.act_time = 100;
    one_pos.pos_name = new_pos_name;

    pos.name = new_pos_name;
    CPosListWidget *pCur_PosWidget = (CPosListWidget *) m_pPosListWidget->itemWidget(m_pPosListWidget->currentItem());
    string pos_name = pCur_PosWidget->pos_name_;
    pos.pose_info = pos_map_[pos_name].pose_info;
    int id = pCur_PosWidget->m_id->text().toInt();
    act_map_[act_name].poses.insert(act_map_[act_name].poses.begin() + id - 1, one_pos);

    if (!exist)
    {
        pos_map_[new_pos_name] = pos;
    }

    updatePosList(act_name);
}

void ActionDebuger::procButtonInsertPosBack()
{
    if (m_pActListWidget->currentItem() == nullptr)
    {
        QMessageBox::warning(this, "Error", "No act select!");
        return;
    }

    string act_name = m_pActListWidget->currentItem()->text().toStdString();

    bool ok;
    string new_pos_name = QInputDialog::getText(this, tr("pos name"), tr("input pos name:"), QLineEdit::Normal, nullptr, &ok).toStdString();

    if (!ok || new_pos_name.empty())
    {
        return;
    }

    bool exist = false;
    auto &pos_map_ = action_eng_->get_pos_map();
    auto &act_map_ = action_eng_->get_act_map();
    if (pos_map_.find(new_pos_name) != pos_map_.end())
    {
        exist = true;
        QMessageBox::StandardButton reply = QMessageBox::question(this, "Warning", "pos: " + QString::fromStdString(new_pos_name) + "  exists, use it?",
                                            QMessageBox::Yes | QMessageBox::No, QMessageBox::Yes);

        if (reply != QMessageBox::StandardButton::Yes)
        {
            return;
        }
    }

    RobotPos pos;
    RobotOnePos one_pos;
    one_pos.act_time = 100;
    one_pos.pos_name = new_pos_name;

    pos.name = new_pos_name;

    if (m_pPosListWidget->currentItem() == nullptr)
    {
        RobotPose pose = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

        for (int i = 1; i <= 6; i++)
        {
            pos.pose_info[static_cast<RobotMotion >(i)] = pose;
        }

        act_map_[act_name].poses.push_back(one_pos);
    }
    else
    {
        CPosListWidget *pCur_PosWidget = (CPosListWidget *) m_pPosListWidget->itemWidget(m_pPosListWidget->currentItem());
        string pos_name = pCur_PosWidget->pos_name_;
        pos.pose_info = pos_map_[pos_name].pose_info;
        int id = pCur_PosWidget->m_id->text().toInt();
        act_map_[act_name].poses.insert(act_map_[act_name].poses.begin() + id, one_pos);
    }

    if (!exist)
    {
        pos_map_[new_pos_name] = pos;
    }

    updatePosList(act_name);
}

void ActionDebuger::procButtonDeletePos()
{
    if (m_pPosListWidget->currentItem() == nullptr)
    {
        QMessageBox::warning(this, "Error", "No pos select!");
        return;
    }

    CPosListWidget *pCur_PosWidget = (CPosListWidget *) m_pPosListWidget->itemWidget(m_pPosListWidget->currentItem());
    auto &pos_map_ = action_eng_->get_pos_map();
    auto &act_map_ = action_eng_->get_act_map();
    int id = pCur_PosWidget->m_id->text().toInt();
    string act_name = m_pActListWidget->currentItem()->text().toStdString();
    act_map_[act_name].poses.erase(act_map_[act_name].poses.begin() + id - 1);
    removeUnusedPos();
    updatePosList(act_name);
}

void ActionDebuger::procButtonSavePos()
{
    if (m_pPosListWidget->currentItem() == nullptr)
    {
        QMessageBox::warning(this, "Error", "No pos select!");
        return;
    }

    CPosListWidget *pCur_PosWidget = (CPosListWidget *) m_pPosListWidget->itemWidget(m_pPosListWidget->currentItem());
    string pos_name = pCur_PosWidget->pos_name_;
    QMessageBox::StandardButton reply = QMessageBox::question(this, "Warning", "current pos is: " + QString::fromStdString(pos_name) + ", save?",
                                        QMessageBox::Yes | QMessageBox::No, QMessageBox::Yes);

    if (reply != QMessageBox::StandardButton::Yes)
    {
        return;
    }
    auto &pos_map_ = action_eng_->get_pos_map();
    auto &act_map_ = action_eng_->get_act_map();
    pos_map_[pos_name].pose_info = pose_map_;
    pos_saved = true;
    save(act_file_, act_map_, pos_map_);
}

void ActionDebuger::procButtonDeleteAction()
{
    if (m_pActListWidget->currentItem() == nullptr)
    {
        QMessageBox::warning(this, "Error", "No act select!");
        return;
    }
    auto &pos_map_ = action_eng_->get_pos_map();
    auto &act_map_ = action_eng_->get_act_map();
    string act_name = m_pActListWidget->currentItem()->text().toStdString();
    auto iter = act_map_.begin();

    while (iter != act_map_.end())
    {
        if (iter->first == act_name)
        {
            act_map_.erase(iter);
            break;
        }

        iter++;
    }

    removeUnusedPos();
    initActs();
}

void ActionDebuger::procButtonSaveAction()
{
    QMessageBox::StandardButton reply = QMessageBox::question(this, "Warning", "all action data will be written into file, save?",
                                        QMessageBox::Yes | QMessageBox::No, QMessageBox::Yes);

    if (reply != QMessageBox::StandardButton::Yes)
    {
        return;
    }
    auto &pos_map_ = action_eng_->get_pos_map();
    auto &act_map_ = action_eng_->get_act_map();
    save(act_file_, act_map_, pos_map_);
}

void ActionDebuger::procButtonAddAction()
{
    bool ok;
    string name = QInputDialog::getText(this, tr("act name"), tr("input act name:"), QLineEdit::Normal, nullptr, &ok).toStdString();

    if (name.empty())
    {
        return;
    }
    auto &pos_map_ = action_eng_->get_pos_map();
    auto &act_map_ = action_eng_->get_act_map();
    if (ok)
    {
        if (act_map_.find(name) != act_map_.end())
        {
            QMessageBox::warning(this, "Waring", "the name had been used", QMessageBox::Yes | QMessageBox::No, QMessageBox::Yes);
            return;
        }

        RobotAct act;
        act.name = name;
        act_map_[name] = act;
        initActs();
    }
}

void ActionDebuger::removeUnusedPos()
{
    bool fd;
    auto &pos_map_ = action_eng_->get_pos_map();
    auto &act_map_ = action_eng_->get_act_map();
    auto p_iter = pos_map_.begin();

    while (p_iter != pos_map_.end())
    {
        fd = false;

        for (auto &act : act_map_)
        {
            for (auto &p : act.second.poses)
            {
                if (p.pos_name == p_iter->first)
                {
                    fd = true;
                    break;
                }
            }

            if (fd)
            {
                break;
            }
        }

        if (!fd)
        {
            p_iter = pos_map_.erase(p_iter);
        }
        else
        {
            p_iter++;
        }
    }
}

void ActionDebuger::procButtonRunPos()
{
    if (m_pPosListWidget->currentItem() == nullptr)
    {
        QMessageBox::warning(this, "Error", "No pos select!");
        return;
    }

    CPosListWidget *pCur_PosWidget = (CPosListWidget *) 
        m_pPosListWidget->itemWidget(m_pPosListWidget->currentItem());
    int id = pCur_PosWidget->m_id->text().toInt();
    string act_name = m_pActListWidget->currentItem()->text().toStdString();
    common::AddAngles add;
    add.request.angles = action_eng_->runAction(act_name, id);
    ros::service::call("/debug/action/run", add);
}

void ActionDebuger::procButtonWebots()
{
    string act_name = m_pActListWidget->currentItem()->text().toStdString();
    auto &act_map_ = action_eng_->get_act_map();
    auto &pos_map_ = action_eng_->get_pos_map();
    RobotAct act = act_map_[act_name];
    std::cout<<"***********************\n"<< act_name<<std::endl;
    for(int i=0; i<act.poses.size(); i++)
    {
        auto &pose = act.poses[i];
        pose_map_ = pos_map_[pose.pos_name].pose_info;
        updateSlider(static_cast<int>(motion_));
        turn_joint();
        std::cout<< pose.act_time<<std::endl;
        for(int i=11; i<23; i++)
        {
            std::cout<<deg2rad(joint_degs_[i]);
            if(i==16||i==22)
                std::cout<<std::endl;
            else
                std::cout<<' ';
        }
        for(int i=7; i<11; i++)
        {
            std::cout<<deg2rad(joint_degs_[i]);
            std::cout<<' ';
        }
        std::cout<<std::endl;
        std::cout<<std::endl;
    }
}

void ActionDebuger::closeEvent(QCloseEvent *event)
{
    QMessageBox::StandardButton reply = QMessageBox::question(this, "Warning", "write action data into file?",
                                        QMessageBox::Yes | QMessageBox::No, QMessageBox::Yes);
    auto &pos_map_ = action_eng_->get_pos_map();
    auto &act_map_ = action_eng_->get_act_map();
    if (reply == QMessageBox::StandardButton::Yes)
    {
        save(act_file_, act_map_, pos_map_);
    }
}