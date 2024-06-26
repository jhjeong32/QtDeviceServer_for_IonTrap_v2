# -*- coding: utf-8 -*-
"""
Created on Thu Oct 28 11:04:18 2021

@author: QCP32
"""

class client_gui_theme_base():
    
    _themes = ["black", "white"]
    _theme = "white"
    
    _menubar_stylesheet = {"white": "",
                           "black": """
                                    QMenuBar{background-color: rgb(40, 40, 40); color: rgb(180, 180, 180);}
                                    QMenuBar::item{background-color: rgb(40, 40, 40); color: rgb(180, 180, 180);}
                                    QMenuBar::item::selected{background-color: rgb(200, 95, 10); color: rgb(180, 180, 180);}
                                    QMenu{background-color: rgb(40, 40, 40); color: rgb(180, 180, 180);}
                                    QMenu::item::selected{background-color: rgb(200, 95, 10); color: rgb(180, 180, 180);}
                                     """
                                     }
    
    _tabbar_stylesheet = {"white": "",
                          "black": """ 
                                    QTabBar {border-radius:5px;}
                                    QTabBar::tab:selected {background:rgb(40, 40, 40); color:rgb(180, 180, 180);}
                                    QTabBar::tab:hover {background:rgb(180, 90, 8); color:rgb(180, 180, 180);}
                                    QTabBar::tab{background:rgb(70, 70, 70); color:rgb(180, 180, 180);}
                                    QTabWidget::pane { border: 0; }
                                    QTabWidget>QWidget>QWidget{background:rgb(80, 80, 80);}
                                    """
                                    }
                                     
    _mainwindow_stylesheet = {"white": "",
                              "black": "background-color:rgb(80, 80, 80); color:rgb(180, 180, 180)"}
    
    _statusbar_stylesheet = {"white": "",
                              "black": "background-color:rgb(80, 80, 80); color:rgb(180, 180, 180)"}
    
    _pushbutton_stylesheet = {"white": "",
                              "black": "background-color:rgb(140, 140, 140); rgb(180, 180, 180)"}
    
    _pushbutton_checked_stylesheet = {"white": "",
                                      "black": "background-color:rgb(200, 95, 10); rgb(180, 180, 180)"}
    

    _theme_base = {"white": """
                          QWidget{
                                 background-color:rgb(255,255,255);
                                 selection-background-color:rgb(130,150,200);
                                 gridline-color:rgb(120,120,120);
                                 color:rgb(0,0,0);
                                 }
                         QHeaderView{background-color:rgb(210,210,210);}
                         QHeaderView::section{background-color:rgb(210,210,210);}
                         QHeaderView::section::checked{background-color:rgb(130,150,200);color:rgb(255,255,255);}
                         QTableCornerButton::section{background-color:rgb(255,255,255);}
                         QComboBox{background-color:rgb(210,210,210);}
             			 QGroupBox{background-color:rgb(255, 255, 255); color:rgb(0, 0, 0);}
                         QLineEdit{background-color:rgb(210, 210, 210); color:rgb(0, 0, 0); border:none;}
             			 QSlider::handle:horizontal {
                                                     background:rgb(145, 168, 210);
                                                     width: 12px;
                                                     height: 6px;
                                                     border-radius: 2px;
                                                     }
                         QProgressBar{border: 2px solid gray;
                                                             border-radius: 5px;
                                                             text-align: center;
                                                             background-color:rgb(255,255,255);
                                                             color:rgb(0,0,0);
                                                             }
                                                QProgressBar::chunk{background-color: lightblue;
                                                                    width: 10px;
                                                                    margin: 1px;}
                         QPushButton{
                                     border-radius: 5px ;background-color: rgb(180, 180, 180); 
                                     color: rgb(255, 255, 255); color: rgb(255, 255, 255);
                                     border-style: outset; border-bottom: 1px solid;
                                     border-right: 1px solid
                                     }
                         QPushButton:pressed {
                                              border-radius: 5px; background-color: rgb(145, 168, 210);
                                              color: rgb(255, 255, 255);
                                              border-style: outset; border-top: 2px solid; border-left: 2px solid;
                                              }
                         QPushButton:checked {
                                              border-radius: 5px; background-color: rgb(145, 168, 210);
                                              color: rgb(255, 255, 255);
                                              border-style: outset; border-top: 2px solid; border-left: 2px solid;
                                              }
                         QPushButton:hover {
                                           border-radius: 5px; background-color: rgb(145, 168, 210);
                                           color: rgb(255, 255, 255);
                                           border-style: outset; border-bottom: 1px solid; border-right: 1px solid;
                                           }
                         QCheckBox::indicator::unchecked:hover{ border: 1px solid skyblue; background-color:white;}
                         QCheckBox::indicator::checked:hover{ border: 1px solid black; background-color:skyblue;}
                         QRadioButton::indicator::unchecked:hover{ border: 1px solid skyblue; background-color:white; border-radius: 5px;}
                         QRadioButton::indicator::checked:hover{ border: 1px solid black; background-color:skyblue; border-radius: 5px;}
                                  """,
               "black": """
                          QWidget{
                                 background-color:rgb(40,40,40);
                                 selection-background-color:rgb(240,180,60);
                                 color:rgb(180,180,180);
                                 gridline-color:rgb(120,120,120);
                                 } 
                         QProgressBar{border: 2px solid lightgray;
                                     border-radius: 5px;
                                     text-align: center;
                                     background-color:rgb(40,40,40);
                                     color:rgb(255,255,255);
                                     }
                         QProgressBar::chunk{background-color: orange;
                                            width: 10px;
                                            margin: 1px;}
                         QHeaderView{background-color:rgb(40,40,40);}
                         QHeaderView::section{background-color:rgb(80,80,80);}
                         QHeaderView::section::checked{background-color:rgb(210,120,20);color:rgb(255,255,255);}
                         QListWidget{background-color:rgb(40,40,40);}
                         QListWidget::item{background-color:rgb(80,80,80);}
                         QListWidget::item::selection{background-color:rgb(210,120,20);color:rgb(255,255,255);}
                         QTableCornerButton::section{background-color:rgb(80,80,80);}
                         QLineEdit{background-color:rgb(0,0,0);color:rgb(180,180,180);border:none;}
                         QTabBar {border-radus:5px;}
                         QTabBar::tab:selected {background:rgb(40, 40, 40); color:rgb(180, 180, 180);}
                         QTabBar::tab{background:rgb(70, 70, 70); color:rgb(180, 180, 180);}
                         QTabWidget::pane { border: 0; }
                         QTabWidget>QWidget>QWidget{background:rgb(40, 40, 40);}
             			 QSlider::handle:horizontal {
                                                     background:rgb(200, 95, 10);
                                                     width: 12px;
                                                     height: 6px;
                                                     border-radius: 2px;}
             			 QComboBox{
                                 color: rgb(180, 180, 180);
                                 background-color: rgb(80, 80, 80);
                                 selection-background-color: rgb(160, 80, 10);
                                 }
                         QComboBox::hover{
                                         background-color:rgb(160, 80, 10);
                                         color:rgb(180, 180, 180);
                                         }
                         QPushButton {
                                     border-radius: 5px; background-color: rgb(80, 80, 80);
                                     color: rgb(180, 180, 180);
                                     border-style: outset; border-bottom: 1px solid; border-right: 1px solid;
                                     }
                         QPushButton:pressed {
                                              border-radius: 5px; background-color: rgb(200, 95, 10);
                                              color: rgb(180, 180, 180);
                                              border-style: outset; border-top: 3px solid; border-left: 3px solid;
                                              }
                         QPushButton:checked {
                                              border-radius: 5px; background-color: rgb(200, 95, 10);
                                              color: rgb(180, 180, 180);
                                              border-style: outset; border-top: 3px solid; border-left: 3px solid;
                                              }
                         QPushButton:hover {
                                           border-radius: 5px; background-color: rgb(200, 95, 10);
                                           color: rgb(180, 180, 180);
                                           border-style: outset; border-bottom: 1px solid; border-right: 1px solid;
                                           }
                         QCheckBox::indicator::unchecked:hover{ border: 1px solid orange; background-color:white;}
                         QCheckBox::indicator::checked:hover{ border: 1px solid dark-gray; background-color:orange;}
                         QRadioButton::indicator::unchecked:hover{ border: 1px solid orange; background-color:white; border-radius: 5px;}
                         QRadioButton::indicator::checked:hover{ border: 1px solid dark-gray; background-color:orange; border-radius: 5px;}
                         """
                         }