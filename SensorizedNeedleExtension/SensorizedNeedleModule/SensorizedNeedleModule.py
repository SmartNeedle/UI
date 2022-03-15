##=========================================================================

#  Program:   Sensorized Needle Module 2021 - Slicer Module
#  Language:  Python

#  Copyright (c) Brigham and Women's Hospital. All rights reserved.

#  This software is distributed WITHOUT ANY WARRANTY; without even
#  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
#  PURPOSE.  See the above copyright notices for more information.

#  Please see
#    https://github.com/SmartNeedle/SystemIntegration
#  for the detail of the protocol.

#=========================================================================


import os
import unittest
import logging
import vtk, qt, ctk, slicer
from slicer.ScriptedLoadableModule import *
from slicer.util import VTKObservationMixin
import numpy as np


class SensorizedNeedleModule(ScriptedLoadableModule):
  """Uses ScriptedLoadableModule base class, available at:
  https://github.com/Slicer/Slicer/blob/master/Bakse/Python/slicer/ScriptedLoadableModule.py
  """

  def __init__(self, parent):
    ScriptedLoadableModule.__init__(self, parent)
    self.parent.title = "Sensorized Needle Module"
    self.parent.categories = ["IGT"]
    self.parent.dependencies = []
    self.parent.contributors = ["Lisa Mareschal"]
    self.parent.helpText = """
"""
    self.parent.helpText += self.getDefaultModuleDocumentationLink()
    self.parent.acknowledgementText = """
"""
    

class SensorizedNeedleModuleWidget(ScriptedLoadableModuleWidget):
  """Uses ScriptedLoadableModuleWidget base class, available at:
  https://github.com/Slicer/Slicer/blob/master/Base/Python/slicer/ScriptedLoadableModule.py
  """
  
  
  def __init__(self, parent=None):
    ScriptedLoadableModuleWidget.__init__(self, parent)

  def setup(self):
    ScriptedLoadableModuleWidget.setup(self)

    # ----- Slicer - Open IGT Link GUI ------	
    
    # Server collapsible button
    serverCollapsibleButton = ctk.ctkCollapsibleButton()
    serverCollapsibleButton.text = "Start IGTLink Server"
    self.layout.addWidget(serverCollapsibleButton)

    # Layout within the path collapsible button
    serverFormLayout = qt.QGridLayout(serverCollapsibleButton)

    self.snrPortTextboxLabel = qt.QLabel('SNR server port:')
    self.snrPortTextbox = qt.QLineEdit("18944")
    self.snrPortTextbox.setReadOnly(False)
    self.snrPortTextbox.setMaximumWidth(250)

    serverFormLayout.addWidget(self.snrPortTextboxLabel, 0, 0)
    serverFormLayout.addWidget(self.snrPortTextbox, 0, 1)

    self.snrHostnameTextboxLabel = qt.QLabel('SNR hostname:')
    self.snrHostnameTextbox = qt.QLineEdit("localhost")
    self.snrHostnameTextbox.setReadOnly(False)
    self.snrHostnameTextbox.setMaximumWidth(250)
    
    serverFormLayout.addWidget(self.snrHostnameTextboxLabel, 1, 0)
    serverFormLayout.addWidget(self.snrHostnameTextbox, 1, 1)

    # Create server button
    self.createServerButton = qt.QPushButton("Create server")
    self.createServerButton.toolTip = "Create the IGTLink server connection with shell."
    self.createServerButton.enabled = True
    self.createServerButton.setMaximumWidth(250)
    
    serverFormLayout.addWidget(self.createServerButton, 2, 0)
    self.createServerButton.connect('clicked()', self.onCreateServerButtonClicked)

    self.disconnectFromSocketButton = qt.QPushButton("Disconnect from socket")
    self.disconnectFromSocketButton.toolTip = "Disconnect from the socket"
    self.disconnectFromSocketButton.enabled = True
    self.disconnectFromSocketButton.setMaximumWidth(250)

    serverFormLayout.addWidget(self.disconnectFromSocketButton, 2, 1)
    self.disconnectFromSocketButton.connect('clicked()', self.onDisconnectFromSocketButtonClicked)
    
    # Initialize observers Button for PoseArray
    #self.IniButton = qt.QPushButton("Initialize")
    #self.IniButton.toolTip = "Add observers on currentshape pose arrays"
    #self.IniButton.enabled = True
    #self.IniButton.setMaximumWidth(200)
    #serverFormLayout.addWidget(self.IniButton)
    #self.IniButton.connect('clicked()', self.onIniButtonClicked)	
    
    # ----- Publishing commands from Slicer to ROS2 modules GUI------	
    # Outbound commands collapsible button
    outboundCollapsibleButton = ctk.ctkCollapsibleButton()
    outboundCollapsibleButton.text = "Outbound commands"
    #outboundCollapsibleButton.collapsed = True
    self.layout.addWidget(outboundCollapsibleButton)
    
    # Layout within the path collapsible button
    
    self.layout.addWidget(outboundCollapsibleButton)
    outboundLayout = qt.QVBoxLayout(outboundCollapsibleButton)
    
    outboundLayoutTopHome = qt.QFormLayout()
    outboundLayout.addLayout(outboundLayoutTopHome)
    
    # Home Button
    self.HomeButton = qt.QPushButton("HOME")
    self.HomeButton.toolTip = "Publish a command to send robot to home position"
    self.HomeButton.enabled = True
    self.HomeButton.setMaximumWidth(200)
    
    outboundLayoutTopHome.addRow("   Calibrate linear stages: ", self.HomeButton)
    self.HomeButton.connect('clicked()', self.onHomeButtonClicked)	
    
    # Stop Button
    self.StopButton = qt.QPushButton("STOP")
    self.StopButton.toolTip = "Publish a command to stop the robot"
    self.StopButton.enabled = True
    self.StopButton.setMaximumWidth(200)
    
    outboundLayoutTopHome.addRow("   Stop Robot: ", self.StopButton)
    self.StopButton.connect('clicked()', self.onStopButtonClicked)	
    
    # Add point temporary Button
    #self.AddPointButton = qt.QPushButton("ADD POINT")
    #self.AddPointButton.toolTip = "Add a point to Curve"
    #self.AddPointButton.enabled = True
    #self.AddPointButton.setMaximumWidth(200)
    #outboundLayoutTopHome.addRow("   Add point:: ", self.AddPointButton)
    #self.AddPointButton.connect('clicked()', self.onAddPointButtonClicked)	
    
    
    
    # Top layout within the outbound collapsible button
    outboundLayoutTop = qt.QFormLayout()
    outboundLayout.addLayout(outboundLayoutTop)
    
    self.targetPointNodeSelector = slicer.qSlicerSimpleMarkupsWidget()
    self.targetPointNodeSelector.objectName = 'targetPointNodeSelector'
    self.targetPointNodeSelector.toolTip = "Select a fiducial to use as the needle insertion target point."
    self.targetPointNodeSelector.setNodeBaseName("TARGET_POINT")
    self.targetPointNodeSelector.defaultNodeColor = qt.QColor(170,0,0)
    self.targetPointNodeSelector.tableWidget().hide()
    self.targetPointNodeSelector.markupsSelectorComboBox().noneEnabled = False
    self.targetPointNodeSelector.markupsPlaceWidget().placeMultipleMarkups = slicer.qSlicerMarkupsPlaceWidget.ForcePlaceSingleMarkup

    outboundLayoutTop.addRow("   Target point: ", self.targetPointNodeSelector)
    self.parent.connect('mrmlSceneChanged(vtkMRMLScene*)',
                        self.targetPointNodeSelector, 'setMRMLScene(vtkMRMLScene*)')
    # Print values target fiducial values                   
    self.TargetXYZ = qt.QHBoxLayout()
    self.xTargetTextbox = qt.QLineEdit("")
    self.xTargetLabel = qt.QLabel("x: ")
    self.xTargetTextbox.setReadOnly(True)
    self.yTargetTextbox = qt.QLineEdit("")
    self.yTargetLabel = qt.QLabel("y: ")
    self.yTargetTextbox.setReadOnly(True)
    self.zTargetTextbox = qt.QLineEdit("")
    self.zTargetLabel = qt.QLabel("z: ")
    self.zTargetTextbox.setReadOnly(True)
    
    self.TargetXYZ.addWidget(self.xTargetLabel)
    self.TargetXYZ.addWidget(self.xTargetTextbox)
    self.TargetXYZ.addWidget(self.yTargetLabel)
    self.TargetXYZ.addWidget(self.yTargetTextbox)
    self.TargetXYZ.addWidget(self.zTargetLabel)
    self.TargetXYZ.addWidget(self.zTargetTextbox)
    outboundLayoutTop.addRow(qt.QLabel("   Target XYZ [mm]:"),self.TargetXYZ)

    self.targetPointNodeSelector.connect('updateFinished()', self.onTargetPointFiducialChanged)

    # Send Target Point Button
    self.sendTargetPointButton = qt.QPushButton("SEND TARGET POINT")
    self.sendTargetPointButton.toolTip = "Publish point coordinates for target point"
    self.sendTargetPointButton.enabled = True
    self.sendTargetPointButton.setMaximumWidth(200)
    outboundLayoutTop.addRow(self.sendTargetPointButton)
    self.sendTargetPointButton.connect('clicked()', self.onsendTargetPointButtonClicked)	

    #self.skinEntryPointNodeSelector = slicer.qSlicerSimpleMarkupsWidget()
    #self.skinEntryPointNodeSelector.objectName = 'skinEntryPointNodeSelector'
    #self.skinEntryPointNodeSelector.toolTip = "Select a fiducial to use as needle entry point on the skin."
    #self.skinEntryPointNodeSelector.setNodeBaseName("SKIN_ENTRY_POINT")
    #self.skinEntryPointNodeSelector.defaultNodeColor = qt.QColor(170,0,0)
    #self.skinEntryPointNodeSelector.tableWidget().hide()
    #self.skinEntryPointNodeSelector.markupsSelectorComboBox().noneEnabled = False
    #self.skinEntryPointNodeSelector.markupsPlaceWidget().placeMultipleMarkups = slicer.qSlicerMarkupsPlaceWidget.ForcePlaceSingleMarkup

    #outboundLayoutTop.addRow("   Skin entry point: ", self.skinEntryPointNodeSelector)
    #self.parent.connect('mrmlSceneChanged(vtkMRMLScene*)',
    #                    self.skinEntryPointNodeSelector, 'setMRMLScene(vtkMRMLScene*)')
                        
    # Print values target fiducial values                       
    self.SkinEntryXYZ = qt.QHBoxLayout()
    self.xSkinEntryTextbox = qt.QLineEdit("")
    self.xSkinEntryLabel = qt.QLabel("x: ")
    self.xSkinEntryTextbox.setReadOnly(False)
    self.ySkinEntryTextbox = qt.QLineEdit("")
    self.ySkinEntryLabel = qt.QLabel("y: ")
    self.ySkinEntryTextbox.setReadOnly(True)
    self.zSkinEntryTextbox = qt.QLineEdit("")
    self.zSkinEntryLabel = qt.QLabel("z: ")
    self.zSkinEntryTextbox.setReadOnly(False)
    
    self.SkinEntryXYZ.addWidget(self.xSkinEntryLabel)
    self.SkinEntryXYZ.addWidget(self.xSkinEntryTextbox)
    self.SkinEntryXYZ.addWidget(self.ySkinEntryLabel)
    self.SkinEntryXYZ.addWidget(self.ySkinEntryTextbox)
    self.SkinEntryXYZ.addWidget(self.zSkinEntryLabel)
    self.SkinEntryXYZ.addWidget(self.zSkinEntryTextbox)
    outboundLayoutTop.addRow(qt.QLabel("   Skin entry XYZ [mm]:"),self.SkinEntryXYZ)
    
    #self.skinEntryPointNodeSelector.connect('updateFinished()', self.onSkinEntryPointFiducialChanged)
    
    
    # Send Skin Entry Point Button
    self.sendSkinEntryPointButton = qt.QPushButton("SEND SKIN ENTRY POINT")
    self.sendSkinEntryPointButton.toolTip = "Publish point coordinates for skin entry point"
    self.sendSkinEntryPointButton.enabled = False
    self.sendSkinEntryPointButton.setMaximumWidth(200)
    outboundLayoutTop.addRow("   Publish Skin Entry coordinates : ",self.sendSkinEntryPointButton)
    self.sendSkinEntryPointButton.connect('clicked()', self.onsendSkinEntryPointButtonClicked)
    
    # Move to skin entry aligned with target point Button
    self.OriginButton = qt.QPushButton("ORIGIN")
    self.OriginButton.toolTip = "Publish a command to move robot to entry position (aligned with target point)"
    self.OriginButton.enabled = False
    self.OriginButton.setMaximumWidth(200)
    outboundLayoutTop.addRow("   Move linear stages to origin: ", self.OriginButton)
    self.OriginButton.connect('clicked()', self.onOriginButtonClicked)	
    
    
    # Visibility icon planned path needle
    self.plannedPathNeedleVisibleButton = qt.QPushButton()
    eyeIconInvisible = qt.QPixmap(":/Icons/Small/SlicerInvisible.png")
    self.plannedPathNeedleVisibleButton.setIcon(qt.QIcon(eyeIconInvisible))
    self.plannedPathNeedleVisibleButton.setFixedWidth(25)
    self.plannedPathNeedleVisibleButton.setCheckable(True)
    # planningLayoutTop.addWidget(self.targetNeedleVisibleButton, 3, 1)
    outboundLayoutTop.addRow(" ", self.plannedPathNeedleVisibleButton)
    self.plannedPathNeedleVisibleButton.connect('clicked()', self.onPlannedPathNeedleVisibleButtonClicked)	
    
    # ----- Feedback from ROS2 modules GUI------	
    # Trajectory estimation model collapsible button
    estimationModelCollapsibleButton = ctk.ctkCollapsibleButton()
    estimationModelCollapsibleButton.text = "INSTRUCTIONS from Trajectory estimation model"
    self.layout.addWidget(estimationModelCollapsibleButton)

    # Layout within the path collapsible button
    
    modelFormLayout = qt.QFormLayout(estimationModelCollapsibleButton)
    
    self.instructionTextbox = qt.QLineEdit("")
    self.instructionTextbox.setReadOnly(True)
    self.instructionTextbox.setFixedWidth(200)
    modelFormLayout.addRow("Instruction:", self.instructionTextbox)

    self.dzTextbox = qt.QLineEdit("No dz value received")
    self.dzTextbox.setReadOnly(True)
    self.dzTextbox.setFixedWidth(200)
    modelFormLayout.addRow("dz:", self.dzTextbox)
    
    self.dthetaTextbox = qt.QLineEdit("No dθ value received")
    self.dthetaTextbox.setReadOnly(True)
    self.dthetaTextbox.setFixedWidth(200)
    modelFormLayout.addRow("dθ:", self.dthetaTextbox)
    
    
    # Needle Guide state collapsible button
    NeedleGuideCollapsibleButton = ctk.ctkCollapsibleButton()
    NeedleGuideCollapsibleButton.text = "Needle guide robot state"
    self.layout.addWidget(NeedleGuideCollapsibleButton)

    # Layout within the path collapsible button
    NeedleGuideFormLayout = qt.QFormLayout(NeedleGuideCollapsibleButton)
    
    #self.poseTextbox = qt.QLineEdit("No needle pose received")
    #self.poseTextbox.setReadOnly(True)
    #self.poseTextbox.setFixedWidth(400)
    #NeedleGuideFormLayout.addRow("Needle pose received:", self.poseTextbox)
    
    self.xTextbox = qt.QLineEdit("No x position received")
    self.xTextbox.setReadOnly(True)
    self.xTextbox.setFixedWidth(200)
    
    self.zTextbox = qt.QLineEdit("No z position received")
    self.zTextbox.setReadOnly(True)
    self.zTextbox.setFixedWidth(200)
    
    self.StateXZ = qt.QHBoxLayout()
    self.StateXZ.addWidget(self.xTextbox)
    self.StateXZ.addWidget(self.zTextbox)
    NeedleGuideFormLayout.addRow("X, Z stage positions:", self.StateXZ)
    
    self.yTextbox = qt.QLineEdit("No y position received")
    self.yTextbox.setReadOnly(True)
    self.yTextbox.setFixedWidth(200)
    
    self.thetaTextbox = qt.QLineEdit("No θ position received")
    self.thetaTextbox.setReadOnly(True)
    self.thetaTextbox.setFixedWidth(200)
    
    self.StateYTheta = qt.QHBoxLayout()
    self.StateYTheta.addWidget(self.yTextbox)
    self.StateYTheta.addWidget(self.thetaTextbox)
    NeedleGuideFormLayout.addRow("Depth Y and rotation θ:", self.StateYTheta)
    
    # ----- Feedback from Imager GUI------	
    # Sensorized needle feedback collapsible button
    sensorizedNeedleCollapsibleButton = ctk.ctkCollapsibleButton()
    sensorizedNeedleCollapsibleButton.text = "Sensorized needle feedback"
    self.layout.addWidget(sensorizedNeedleCollapsibleButton)
    
    
    NeedleShapeGridLayout = qt.QFormLayout(sensorizedNeedleCollapsibleButton)
    
    # Print values target fiducial values                       
    self.NeedleEndXYZ = qt.QHBoxLayout()
    self.xNeedleEndTextbox = qt.QLineEdit("")
    self.xNeedleEndLabel = qt.QLabel("x: ")
    self.xNeedleEndTextbox.setReadOnly(True)
    self.yNeedleEndTextbox = qt.QLineEdit("")
    self.yNeedleEndLabel = qt.QLabel("y: ")
    self.yNeedleEndTextbox.setReadOnly(True)
    self.zNeedleEndTextbox = qt.QLineEdit("")
    self.zNeedleEndLabel = qt.QLabel("z: ")
    self.zNeedleEndTextbox.setReadOnly(True)
    
    self.NeedleEndXYZ.addWidget(self.xNeedleEndLabel)
    self.NeedleEndXYZ.addWidget(self.xNeedleEndTextbox)
    self.NeedleEndXYZ.addWidget(self.yNeedleEndLabel)
    self.NeedleEndXYZ.addWidget(self.yNeedleEndTextbox)
    self.NeedleEndXYZ.addWidget(self.zNeedleEndLabel)
    self.NeedleEndXYZ.addWidget(self.zNeedleEndTextbox)
    NeedleShapeGridLayout.addRow(qt.QLabel("   End point position of the needle shape:"),self.NeedleEndXYZ)
    
    # Define empty Nodes 
    self.PlannedPathTransform = None
    self.XStageTransform = None
    #XStageMatrix = vtk.vtkMatrix4x4()
    #XStageMatrix.Identity()
    #self.XStageTransform.SetMatrixTransformToParent(XStageMatrix)
    
    #row = 4
    #column = 4
    #self.ShapetableWidget = qt.QTableWidget(row, column)
    #self.ShapetableWidget.setMinimumHeight(95)
    #self.ShapetableWidget.verticalHeader().hide() # Remove line numbers
    #self.ShapetableWidget.horizontalHeader().hide() # Remove column numbers
    #self.ShapetableWidget.setEditTriggers(qt.QTableWidget.NoEditTriggers) # Make table read-only
    #horizontalheader = self.ShapetableWidget.horizontalHeader()
    #horizontalheader.setSectionResizeMode(0, qt.QHeaderView.Stretch)
    #horizontalheader.setSectionResizeMode(1, qt.QHeaderView.Stretch)
    #horizontalheader.setSectionResizeMode(2, qt.QHeaderView.Stretch)
    #horizontalheader.setSectionResizeMode(3, qt.QHeaderView.Stretch)

    #verticalheader = self.ShapetableWidget.verticalHeader()
    #verticalheader.setSectionResizeMode(0, qt.QHeaderView.Stretch)
    #verticalheader.setSectionResizeMode(1, qt.QHeaderView.Stretch)
    #verticalheader.setSectionResizeMode(2, qt.QHeaderView.Stretch)
    #verticalheader.setSectionResizeMode(3, qt.QHeaderView.Stretch)
    #ShapetableWidgetLabel = qt.QLabel("   End point position of the needle shape:")
    #NeedleShapeGridLayout.addWidget(ShapetableWidgetLabel, 3, 0)
    #NeedleShapeGridLayout.addWidget(self.ShapetableWidget, 3, 1)
    

    
  # ----- Functions to establish OpenIGTL connection------	  
  def onCreateServerButtonClicked(self):
    snrPort = self.snrPortTextbox.text
    snrHostname = self.snrHostnameTextbox.text
    print("Slicer-side port number: ", snrPort)
    #VisualFeedback: color in gray when server is created
    self.snrPortTextboxLabel.setStyleSheet('color: rgb(195,195,195)')
    self.snrHostnameTextboxLabel.setStyleSheet('color: rgb(195,195,195)')
    self.snrPortTextbox.setStyleSheet("""QLineEdit { background-color: white; color: rgb(195,195,195) }""")
    self.snrHostnameTextbox.setStyleSheet("""QLineEdit { background-color: white; color: rgb(195,195,195) }""")
    # Disable textboxes
    self.snrPortTextbox.setReadOnly(True)
    self.snrHostnameTextbox.setReadOnly(True)
    # Initialize the IGTLink Slicer-side server component
    self.openIGTNode = slicer.vtkMRMLIGTLConnectorNode()
    slicer.mrmlScene.AddNode(self.openIGTNode)
    self.openIGTNode.SetTypeServer(int(snrPort))
    self.openIGTNode.Start()
    print("openIGTNode: ", self.openIGTNode)
    self.IGTActive = True
    
    # Make a node for each parameter type
    #ReceivedStringMsg = slicer.vtkMRMLTextNode()
    #ReceivedStringMsg.SetName("StringMessage")
    #slicer.mrmlScene.AddNode(ReceivedStringMsg)
    
    ReceivedNeedlePose = slicer.vtkMRMLTextNode()
    ReceivedNeedlePose.SetName("/stage/state/needle")
    slicer.mrmlScene.AddNode(ReceivedNeedlePose)
    
    ReceivedStringNewShape = slicer.vtkMRMLTextNode()
    ReceivedStringNewShape.SetName("NewShape")
    slicer.mrmlScene.AddNode(ReceivedStringNewShape)
    
    #ReceivedNeedleShape0 = slicer.vtkMRMLLinearTransformNode()
    #ReceivedNeedleShape0.SetName("currentshape_0")
    #slicer.mrmlScene.AddNode(ReceivedNeedleShape0)
    
    XStageTransform = slicer.vtkMRMLLinearTransformNode()
    XStageTransform.SetName("XStageTransform")
    slicer.mrmlScene.AddNode(XStageTransform)
    
    #Add node for Needle Shape curve
    CurveNeedleShapeNode = slicer.vtkMRMLMarkupsCurveNode()
    CurveNeedleShapeNode.SetName("CurveNeedleShape")
    slicer.mrmlScene.AddNode(CurveNeedleShapeNode)  
    
    #Add node for Needle Shape fiducials
    #FiducialsNeedleShapeNode = slicer.vtkMRMLMarkupsFiducialNode()
    #FiducialsNeedleShapeNode.SetName("FiducialsNeedleShape")
    #slicer.mrmlScene.AddNode(FiducialsNeedleShapeNode)  
    
    # Add observers on the message type nodes
    ReceivedStringNewShape.AddObserver(slicer.vtkMRMLTextNode.TextModifiedEvent, self.onNeedleShapeNodeModified)
    
    ReceivedNeedlePose.AddObserver(slicer.vtkMRMLTextNode.TextModifiedEvent, self.onNeedlePoseNodeModified)
    #ReceivedNeedleShape0.AddObserver(slicer.vtkMRMLTransformNode.TransformModifiedEvent, self.onNeedleShapeNodeModified)
    
  def onDisconnectFromSocketButtonClicked(self):
    self.openIGTNode.Stop()
    #VisualFeedback: color in black when socket is disconnected
    self.snrPortTextboxLabel.setStyleSheet('color: black')
    self.snrHostnameTextboxLabel.setStyleSheet('color: black')
    self.snrPortTextbox.setStyleSheet("""QLineEdit { background-color: white; color: black }""")
    self.snrHostnameTextbox.setStyleSheet("""QLineEdit { background-color: white; color: black }""")
    # Enable textboxes
    self.snrPortTextbox.setReadOnly(False)
    self.snrHostnameTextbox.setReadOnly(False)
  
  #def onIniButtonClicked(self):
    #ReceivedNeedleShape0 = slicer.mrmlScene.GetFirstNodeByName("currentshape_0")
    #ReceivedNeedleShape0.AddObserver(slicer.vtkMRMLTransformNode.TransformModifiedEvent, self.onNeedleShapeNodeModified)
    
  # ----- Functions to set target and skin entry points ------	  
    
  def onTargetPointFiducialChanged(self):
    targetPointNode = self.targetPointNodeSelector.currentNode()
    if targetPointNode is not None:
    	if not targetPointNode.GetNumberOfControlPoints() == 0:
    		targetCoordinatesRAS = targetPointNode.GetNthControlPointPositionVector(0)
    		self.xTargetTextbox.setText(round(targetCoordinatesRAS[0],2))
    		self.yTargetTextbox.setText(round(targetCoordinatesRAS[1],2))
    		self.zTargetTextbox.setText(round(targetCoordinatesRAS[2],2))
    		
  #def onSkinEntryPointFiducialChanged(self):
    #skinEntryPointNode = self.skinEntryPointNodeSelector.currentNode()
   # if skinEntryPointNode is not None:
      #if not skinEntryPointNode.GetNumberOfControlPoints() == 0:
      #  if self.PlannedPathTransform:
         # slicer.mrmlScene.RemoveNode(self.PlannedPathTransform)
         # self.PlannedPathTransform = None
       # self.PlannedPathTransform = slicer.vtkMRMLLinearTransformNode()
       # self.PlannedPathTransform.SetName("PlannedPathTransform")
       # slicer.mrmlScene.AddNode(self.PlannedPathTransform)
        #self.PlannedPathTransform.AddObserver(slicer.vtkMRMLTransformNode.TransformModifiedEvent, self.onPlannedPathTransformNodeModified)
        
        #skinEntryCoordinatesRAS = skinEntryPointNode.GetNthControlPointPositionVector(0)
        #self.xSkinEntryTextbox.setText(round(skinEntryCoordinatesRAS[0],2))
       # self.ySkinEntryTextbox.setText(round(skinEntryCoordinatesRAS[1],2))
       # self.zSkinEntryTextbox.setText(round(skinEntryCoordinatesRAS[2],2))
        #skinEntryPointMatrix = vtk.vtkMatrix4x4()
       # skinEntryPointMatrix.Identity()
        #skinEntryPointMatrix.SetElement(0,3,skinEntryCoordinatesRAS[0])
        #skinEntryPointMatrix.SetElement(1,3,skinEntryCoordinatesRAS[1])
        #skinEntryPointMatrix.SetElement(2,3,skinEntryCoordinatesRAS[2])
        #self.PlannedPathTransform.SetMatrixTransformToParent(skinEntryPointMatrix)
       # if slicer.mrmlScene.GetFirstNodeByName("PlannedPathNeedle") is not None:
        #  PointerNodeToRemove = slicer.mrmlScene.GetFirstNodeByName("PlannedPathNeedle")
      #    slicer.mrmlScene.RemoveNode(PointerNodeToRemove)      
       # self.AddPointerModel("PlannedPathNeedle")
      #  TransformNodeToDisplay = slicer.mrmlScene.GetFirstNodeByName("PlannedPathTransform")
       # locatorModelNode = slicer.mrmlScene.GetFirstNodeByName("PlannedPathNeedle")
       # locatorModelNode.SetAndObserveTransformNodeID(TransformNodeToDisplay.GetID())
        
        
        
  #def onPlannedPathTransformNodeModified(self, unusedArg2=None, unusedArg3=None):
    #print("Nouvelle planned path ")
    # Update targetTableWidget when the targetTransform is modified
    #targetTransformMatrix = vtk.vtkMatrix4x4()
    #self.plannedTargetTransform.GetMatrixTransformToParent(targetTransformMatrix)
    #nbRows = self.targetTableWidget.rowCount
    #nbColumns = self.targetTableWidget.columnCount
    #for i in range(nbRows):
      #for j in range(nbColumns):
        #self.targetTableWidget.setItem(i , j, qt.QTableWidgetItem(str(round(targetTransformMatrix.GetElement(i,j),2))))
        
  def AddPointerModel(self, pointerNodeName):   
    self.cyl = vtk.vtkCylinderSource()
    self.cyl.SetRadius(1.5)
    self.cyl.SetResolution(50)
    self.cyl.SetHeight(100)

    mapper = vtk.vtkPolyDataMapper()
    mapper.SetInputConnection(self.cyl.GetOutputPort())

    actor = vtk.vtkActor()
    actor.SetOrientation(0,180,0)
    actor.SetMapper(mapper)

    node = self.cyl.GetOutput()
    locatorModelNode = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLModelNode", pointerNodeName)
    locatorModelNode.SetAndObservePolyData(node)
    locatorModelNode.CreateDefaultDisplayNodes()
    locatorModelNode.SetDisplayVisibility(True)
    # Set needle model color based on the type of needle (planned target, reachable target, or current position)
    if pointerNodeName == "PlannedPathNeedle": 
      locatorModelNode.GetDisplayNode().SetColor(0.80,0.80,0.80) # grey
    elif pointerNodeName == "ShapeNeedle":
      locatorModelNode.GetDisplayNode().SetColor(0.48,0.75,0.40) # green
    #else: #pointerNodeName == "CurrentPositionNeedle"
     # locatorModelNode.GetDisplayNode().SetColor(0.40,0.48,0.75) # blue
    self.cyl.Update()

    #Rotate cylinder
    transformFilter = vtk.vtkTransformPolyDataFilter()
    transform = vtk.vtkTransform()
    transform.RotateX(90.0)
    transform.Translate(0.0, -50.0, 0.0)
    transform.Update()
    transformFilter.SetInputConnection(self.cyl.GetOutputPort())
    transformFilter.SetTransform(transform)

    self.sphere = vtk.vtkSphereSource()
    self.sphere.SetRadius(3.0)
    self.sphere.SetCenter(0, 0, 0)

    self.append = vtk.vtkAppendPolyData()
    self.append.AddInputConnection(self.sphere.GetOutputPort())
    self.append.AddInputConnection(transformFilter.GetOutputPort())
    self.append.Update()

    locatorModelNode.SetAndObservePolyData(self.append.GetOutput())
    #self.treeView.setMRMLScene(slicer.mrmlScene)
  		
    		
  def onPlannedPathNeedleVisibleButtonClicked(self):
    # If button is checked
    if (self.plannedPathNeedleVisibleButton.isChecked()):
      #if slicer.mrmlScene.GetFirstNodeByName("PlannedPathNeedle") is not None:
        #PointerNodeToRemove = slicer.mrmlScene.GetFirstNodeByName("PlannedPathNeedle")
        #slicer.mrmlScene.RemoveNode(PointerNodeToRemove)
      eyeIconVisible = qt.QPixmap(":/Icons/Small/SlicerVisible.png")
      self.plannedPathNeedleVisibleButton.setIcon(qt.QIcon(eyeIconVisible))
      #self.AddPointerModel("PlannedPathNeedle")
      #TransformNodeToDisplay = slicer.mrmlScene.GetFirstNodeByName("PlannedPathTransform")
      #locatorModelNode = slicer.mrmlScene.GetFirstNodeByName("PlannedPathNeedle")
      #locatorModelNode.SetAndObserveTransformNodeID(TransformNodeToDisplay.GetID())
    # If it is unchecked
    else:
      eyeIconInvisible = qt.QPixmap(":/Icons/Small/SlicerInvisible.png")
      self.plannedPathNeedleVisibleButton.setIcon(qt.QIcon(eyeIconInvisible))
      PointerNodeToRemove = slicer.mrmlScene.GetFirstNodeByName("PlannedPathNeedle")
      slicer.mrmlScene.RemoveNode(PointerNodeToRemove)
  def onHomeButtonClicked(self):
    print("Sending home command")
    #Publish home command   
    # Send stringMessage containing the command "HOME" to the script via IGTLink
    homeNode = slicer.vtkMRMLTextNode()
    homeNode.SetName("HOME")
    homeNode.SetText("HOME")
    homeNode.SetEncoding(3)
    slicer.mrmlScene.AddNode(homeNode)
    self.openIGTNode.RegisterOutgoingMRMLNode(homeNode)
    self.openIGTNode.PushNode(homeNode) 
    
  def onStopButtonClicked(self):
    print("Sending ABORT command")
    #Publish ABORT command   
    # Send stringMessage containing the command "HOME" to the script via IGTLink
    stopNode = slicer.vtkMRMLTextNode()
    stopNode.SetName("STOP")
    stopNode.SetText("ABORT")
    stopNode.SetEncoding(3)
    slicer.mrmlScene.AddNode(stopNode)
    self.openIGTNode.RegisterOutgoingMRMLNode(stopNode)
    self.openIGTNode.PushNode(stopNode) 
     
  #def onAddPointButtonClicked(self):
    #print("Sending home command")
    #TODO 
    #Publish home command  
    #CurveNeedleShapeNode = slicer.mrmlScene.GetFirstNodeByName("CurveNeedleShape")        
    # Using above first method to create a 2D array
    
    #newPoint = [98, 100, 110]
    #print(newPoint)  
    
    #CurveNeedleShapeNode.AddControlPoint(vtk.vtkVector3d(newPoint[0],newPoint[1],newPoint[2]))   
    
  def onOriginButtonClicked(self):
    print("Moving robot to origin")
    # Send stringMessage containing the command "ORIGIN;xskinEntry;zSkinEntry" to the script via IGTLink
    originNode = slicer.vtkMRMLTextNode()
    originNode.SetName("ORIGIN")
    origin_msg = "ORIGIN;" + self.xSkinEntryTextbox.text + ";" + self.zSkinEntryTextbox.text
    originNode.SetText(origin_msg)
    print("Sending origin msg: ", origin_msg)
    originNode.SetEncoding(3)
    slicer.mrmlScene.AddNode(originNode)
    self.openIGTNode.RegisterOutgoingMRMLNode(originNode)
    self.openIGTNode.PushNode(originNode) 
    
  def onsendTargetPointButtonClicked(self):   
    print("Sending target point coordinates")
    targetMsgNode = slicer.mrmlScene.GetFirstNodeByName("TARGET_POINT")
    self.openIGTNode.RegisterOutgoingMRMLNode(targetMsgNode)
    self.openIGTNode.PushNode(targetMsgNode)
    #Enable Origin button and send skin target button 
    self.sendSkinEntryPointButton.enabled = True
    self.OriginButton.enabled = True
    
    self.xSkinEntryTextbox.setText(self.xTargetTextbox.text)
    self.zSkinEntryTextbox.setText(self.zTargetTextbox.text)
    
  def onsendSkinEntryPointButtonClicked(self):  
    print("Sending skin entry point coordinates")
    skinEntryMsgNode = slicer.mrmlScene.GetFirstNodeByName("SKIN_ENTRY_POINT")
    self.openIGTNode.RegisterOutgoingMRMLNode(skinEntryMsgNode)
    self.openIGTNode.PushNode(skinEntryMsgNode)  
   #TODO Make it aligned with target point and not a point you can click 
   # Maybe define one that is linea, one with 5 degrees angle to the right, one with five degree angle to the left 
   
   # ----- Functions to get needle pose feedback ------
     
  def onNeedlePoseNodeModified(poseNode, unusedArg2=None, unusedArg3=None):
    print("New needle pose was received")
    ReceivedNeedlePose = slicer.mrmlScene.GetFirstNodeByName("/stage/state/needle")
    concatenatePose = ReceivedNeedlePose.GetText()
    #poseNode.poseTextbox.setText(concatenatePose)
    print("Received ", ReceivedNeedlePose.GetText()) 
    
    # Deconcatenate pose values
    delimit = ","
    if(concatenatePose.find(delimit)!=-1): # found delimiter in the string
      dz = concatenatePose[0: concatenatePose.index(delimit)]
      idx = concatenatePose.index(delimit)
      rest_string = concatenatePose[idx +1 :len(concatenatePose)]
      dtheta = rest_string[0: rest_string.index(delimit)]
      idx = rest_string.index(delimit)
      rest_string = rest_string[idx +1 :len(rest_string)]
      x = rest_string[0: rest_string.index(delimit)]
      idx = rest_string.index(delimit)
      rest_string = rest_string[idx +1 :len(rest_string)]
      y = rest_string[0: rest_string.index(delimit)]
      idx = rest_string.index(delimit)
      rest_string = rest_string[idx +1 :len(rest_string)]
      z = rest_string[0: rest_string.index(delimit)]
      idx = rest_string.index(delimit)
      rest_string = rest_string[idx +1 :len(rest_string)]
      theta = rest_string
      
      poseNode.dzTextbox.setText(dz)
      poseNode.dthetaTextbox.setText(dtheta)
      poseNode.xTextbox.setText(x)
      poseNode.yTextbox.setText(y)
      poseNode.zTextbox.setText(z)
      poseNode.thetaTextbox.setText(theta)
      
      # Update the transform of Block X 
      XStageMatrix = vtk.vtkMatrix4x4()
      XStageMatrix.Identity()
      XStageMatrix.SetElement(0,3,float(x))
      XStageMatrix.SetElement(2,3,float(z))
      XStageMatrix.SetElement(1,3,float(y))
      poseNode.XStageTransform = slicer.mrmlScene.GetFirstNodeByName("XStageTransform")
      poseNode.XStageTransform.SetMatrixTransformToParent(XStageMatrix)
      if slicer.mrmlScene.GetFirstNodeByName("XStage") is not None:
        BlockNodeToRemove = slicer.mrmlScene.GetFirstNodeByName("XStage")
        slicer.mrmlScene.RemoveNode(BlockNodeToRemove)      
      poseNode.AddBlockModel("XStage")
      TransformNodeToDisplay = slicer.mrmlScene.GetFirstNodeByName("XStageTransform")
      blockModelNode = slicer.mrmlScene.GetFirstNodeByName("XStage")
      blockModelNode.SetAndObserveTransformNodeID(TransformNodeToDisplay.GetID())
      
  def onNeedleShapeNodeModified(shapeNode, unusedArg2=None, unusedArg3=None): 
    ReceivedStringNewShape = slicer.mrmlScene.GetFirstNodeByName("NewShape")
    concatenateShape = ReceivedStringNewShape.GetText()
    
    # Remove and create new fiducials points for needle shape 	
    #FiducialsNeedleShapeNode = slicer.mrmlScene.GetFirstNodeByName("FiducialsNeedleShape") 
    #slicer.mrmlScene.RemoveNode(FiducialsNeedleShapeNode)
    #FiducialsNeedleShapeNode = slicer.vtkMRMLMarkupsFiducialNode()
    #FiducialsNeedleShapeNode.SetName("FiducialsNeedleShape")
    #slicer.mrmlScene.AddNode(FiducialsNeedleShapeNode)
    
    delimit = ";"
    if(concatenateShape.find(delimit)!=-1): # found delimiter in the string
      nb_shape = concatenateShape[0: concatenateShape.index(delimit)]
      idx = concatenateShape.index(delimit)
      nb_poses = concatenateShape[idx +1 :len(concatenateShape)]
      nb_poses = int(nb_poses)
      print("Needle shape nb:", nb_shape ,"was received and has", nb_poses,"poses") 
    
      # Initialize variables	
      #i = 0;
      pointPositions = np.empty((0, 3), float)
    
      #while  slicer.mrmlScene.GetFirstNodeByName("currentshape_" + str(i)) is not None:
      for i in range (nb_poses):
        ReceivedNeedleShape_temp = slicer.mrmlScene.GetFirstNodeByName("currentshape_" +  str(i))
        transformMatrix_temp = vtk.vtkMatrix4x4()
        ReceivedNeedleShape_temp.GetMatrixTransformToParent(transformMatrix_temp)
        i+=1
        
        # Append new point to array point positions
        pointPositions = np.append(pointPositions, np.array([[transformMatrix_temp.GetElement(0,3), transformMatrix_temp.GetElement(1,3), transformMatrix_temp.GetElement(2,3)]]), axis=0)

      # For list of fiducials  
        #pointsVector = vtk.vtkVector3d()
        #pointsVector.SetX(transformMatrix_temp.GetElement(0,3))
        #pointsVector.SetY(transformMatrix_temp.GetElement(1,3))
        #pointsVector.SetZ(transformMatrix_temp.GetElement(2,3))
        #FiducialsNeedleShapeNode.AddControlPoint(pointsVector)
      # Display the end point transform 
      ReceivedNeedleShape_END = slicer.mrmlScene.GetFirstNodeByName("currentshape_" +  str(nb_poses-1))
      endPointtransformMatrix = vtk.vtkMatrix4x4()
      ReceivedNeedleShape_END.GetMatrixTransformToParent(endPointtransformMatrix)
    
      shapeNode.xNeedleEndTextbox.setText(round(endPointtransformMatrix.GetElement(0,3),2))
      shapeNode.yNeedleEndTextbox.setText(round(endPointtransformMatrix.GetElement(1,3),2))
      shapeNode.zNeedleEndTextbox.setText(round(endPointtransformMatrix.GetElement(2,3),2))
        
      # Update Shape Needle Curve    
      #print(pointPositions)
      print("The nb of points in the Curve is:" , len(pointPositions))
      points = vtk.vtkPoints()
      vtkpointsData = vtk.util.numpy_support.numpy_to_vtk(pointPositions, deep=1)
      points.SetNumberOfPoints(len(pointPositions))
      points.SetData(vtkpointsData)
    
      CurveNeedleShapeNode = slicer.mrmlScene.GetFirstNodeByName("CurveNeedleShape") 
      slicer.mrmlScene.RemoveNode(CurveNeedleShapeNode)
      CurveNeedleShapeNode = slicer.vtkMRMLMarkupsCurveNode()
      CurveNeedleShapeNode.SetName("CurveNeedleShape")
      slicer.mrmlScene.AddNode(CurveNeedleShapeNode) 
      CurveNeedleShapeNode.SetControlPointPositionsWorld(points)      
    
      slicer.util.updateMarkupsControlPointsFromArray(CurveNeedleShapeNode, pointPositions)
    else: 
      print("Error: No indication on nb of needle shape poses sent")    
    
  def AddBlockModel(self, blockNodeName):   
    self.Xrec = vtk.vtkCubeSource()
    self.Xrec.SetXLength(60)
    self.Xrec.SetYLength(30)
    self.Xrec.SetZLength(40)
   
    mapper = vtk.vtkPolyDataMapper()
    mapper.SetInputConnection(self.Xrec.GetOutputPort())

    actor = vtk.vtkActor()
    actor.SetOrientation(0,0,90)
    actor.SetMapper(mapper)

    node = self.Xrec.GetOutput()
    locatorModelNode = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLModelNode", blockNodeName)
    locatorModelNode.SetAndObservePolyData(node)
    locatorModelNode.CreateDefaultDisplayNodes()
    locatorModelNode.SetDisplayVisibility(True)
    
    # Set needle model color based on the type of needle (planned target, reachable target, or current position)
    if blockNodeName == "XStage": 
      locatorModelNode.GetDisplayNode().SetColor(0.44,0.80,0.85) # light blue
    #elif blockNodeName == "ShapeNeedle":
      #locatorModelNode.GetDisplayNode().SetColor(0.48,0.75,0.40) # green
    #else: #pointerNodeName == "CurrentPositionNeedle"
     # locatorModelNode.GetDisplayNode().SetColor(0.40,0.48,0.75) # blue
    self.Xrec.Update()

    #Rotate Xrec
    transformFilter = vtk.vtkTransformPolyDataFilter()
    transform = vtk.vtkTransform()
    transform.RotateX(90.0)
    transform.Translate(0.0, -50.0, 0.0)
    transform.Update()
    transformFilter.SetInputConnection(self.Xrec.GetOutputPort())
    transformFilter.SetTransform(transform)

    self.Zrec = vtk.vtkCubeSource()
    self.Zrec.SetXLength(30)
    self.Zrec.SetYLength(40)
    self.Zrec.SetZLength(60)

    self.append = vtk.vtkAppendPolyData()
    self.append.AddInputConnection(self.Zrec.GetOutputPort())
    self.append.AddInputConnection(transformFilter.GetOutputPort())
    #self.append.Update()

    #locatorModelNode.SetAndObservePolyData(self.append.GetOutput())
    
    self.Yrec = vtk.vtkCubeSource()
    self.Yrec.SetXLength(10)
    self.Yrec.SetYLength(150)
    self.Yrec.SetZLength(50)

    #self.append = vtk.vtkAppendPolyData()
    self.append.AddInputConnection(self.Yrec.GetOutputPort())
    self.append.AddInputConnection(transformFilter.GetOutputPort())
    self.append.Update()

    locatorModelNode.SetAndObservePolyData(self.append.GetOutput())
    #self.treeView.setMRMLScene(slicer.mrmlScene)  
    
    
    
    
    
    
    
    
    
    
    
  
