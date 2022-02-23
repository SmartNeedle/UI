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
    
    # ----- Publishing commands from Slicer to ROS2 modules GUI------	
    # Outbound commands collapsible button
    outboundCollapsibleButton = ctk.ctkCollapsibleButton()
    outboundCollapsibleButton.text = "Outbound commands to set target and entry point"
    #outboundCollapsibleButton.collapsed = True
    self.layout.addWidget(outboundCollapsibleButton)
    
    # Layout within the path collapsible button
    
    self.layout.addWidget(outboundCollapsibleButton)
    outboundLayout = qt.QVBoxLayout(outboundCollapsibleButton)

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
    outboundLayoutTop.addRow(qt.QLabel("   Target XYZ:"),self.TargetXYZ)

    self.targetPointNodeSelector.connect('updateFinished()', self.onTargetPointFiducialChanged)

    # Send Target Point Button
    self.sendTargetPointButton = qt.QPushButton("SEND TARGET POINT")
    self.sendTargetPointButton.toolTip = "Publish point coordinates for target point"
    self.sendTargetPointButton.enabled = True
    self.sendTargetPointButton.setMaximumWidth(200)
    outboundLayoutTop.addRow(self.sendTargetPointButton)
    self.sendTargetPointButton.connect('clicked()', self.onsendTargetPointButtonClicked)	

    self.skinEntryPointNodeSelector = slicer.qSlicerSimpleMarkupsWidget()
    self.skinEntryPointNodeSelector.objectName = 'skinEntryPointNodeSelector'
    self.skinEntryPointNodeSelector.toolTip = "Select a fiducial to use as needle entry point on the skin."
    self.skinEntryPointNodeSelector.setNodeBaseName("SKIN_ENTRY_POINT")
    self.skinEntryPointNodeSelector.defaultNodeColor = qt.QColor(170,0,0)
    self.skinEntryPointNodeSelector.tableWidget().hide()
    self.skinEntryPointNodeSelector.markupsSelectorComboBox().noneEnabled = False
    self.skinEntryPointNodeSelector.markupsPlaceWidget().placeMultipleMarkups = slicer.qSlicerMarkupsPlaceWidget.ForcePlaceSingleMarkup

    outboundLayoutTop.addRow("   Skin entry point: ", self.skinEntryPointNodeSelector)
    self.parent.connect('mrmlSceneChanged(vtkMRMLScene*)',
                        self.skinEntryPointNodeSelector, 'setMRMLScene(vtkMRMLScene*)')
                        
    # Print values target fiducial values                       
    self.SkinEntryXYZ = qt.QHBoxLayout()
    self.xSkinEntryTextbox = qt.QLineEdit("")
    self.xSkinEntryLabel = qt.QLabel("x: ")
    self.xSkinEntryTextbox.setReadOnly(True)
    self.ySkinEntryTextbox = qt.QLineEdit("")
    self.ySkinEntryLabel = qt.QLabel("y: ")
    self.ySkinEntryTextbox.setReadOnly(True)
    self.zSkinEntryTextbox = qt.QLineEdit("")
    self.zSkinEntryLabel = qt.QLabel("z: ")
    self.zSkinEntryTextbox.setReadOnly(True)
    
    self.SkinEntryXYZ.addWidget(self.xSkinEntryLabel)
    self.SkinEntryXYZ.addWidget(self.xSkinEntryTextbox)
    self.SkinEntryXYZ.addWidget(self.ySkinEntryLabel)
    self.SkinEntryXYZ.addWidget(self.ySkinEntryTextbox)
    self.SkinEntryXYZ.addWidget(self.zSkinEntryLabel)
    self.SkinEntryXYZ.addWidget(self.zSkinEntryTextbox)
    outboundLayoutTop.addRow(qt.QLabel("   Skin entry XYZ:"),self.SkinEntryXYZ)
    
    self.skinEntryPointNodeSelector.connect('updateFinished()', self.onSkinEntryPointFiducialChanged)
    
    
    # Send Skin Entry Point Button
    self.sendSkinEntryPointButton = qt.QPushButton("SEND SKIN ENTRY POINT")
    self.sendSkinEntryPointButton.toolTip = "Publish point coordinates for skin entry point"
    self.sendSkinEntryPointButton.enabled = True
    self.sendSkinEntryPointButton.setMaximumWidth(200)
    outboundLayoutTop.addRow(self.sendSkinEntryPointButton)
    self.sendSkinEntryPointButton.connect('clicked()', self.onsendSkinEntryPointButtonClicked)
    
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
    
    self.poseTextbox = qt.QLineEdit("No needle pose received")
    self.poseTextbox.setReadOnly(True)
    self.poseTextbox.setFixedWidth(400)
    NeedleGuideFormLayout.addRow("Needle pose received:", self.poseTextbox)
    
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
    
    NeedleShapeGridLayout = qt.QGridLayout(sensorizedNeedleCollapsibleButton)
    row = 4
    column = 4
    self.ShapetableWidget = qt.QTableWidget(row, column)
    self.ShapetableWidget.setMinimumHeight(95)
    self.ShapetableWidget.verticalHeader().hide() # Remove line numbers
    self.ShapetableWidget.horizontalHeader().hide() # Remove column numbers
    self.ShapetableWidget.setEditTriggers(qt.QTableWidget.NoEditTriggers) # Make table read-only
    horizontalheader = self.ShapetableWidget.horizontalHeader()
    horizontalheader.setSectionResizeMode(0, qt.QHeaderView.Stretch)
    horizontalheader.setSectionResizeMode(1, qt.QHeaderView.Stretch)
    horizontalheader.setSectionResizeMode(2, qt.QHeaderView.Stretch)
    horizontalheader.setSectionResizeMode(3, qt.QHeaderView.Stretch)

    verticalheader = self.ShapetableWidget.verticalHeader()
    verticalheader.setSectionResizeMode(0, qt.QHeaderView.Stretch)
    verticalheader.setSectionResizeMode(1, qt.QHeaderView.Stretch)
    verticalheader.setSectionResizeMode(2, qt.QHeaderView.Stretch)
    verticalheader.setSectionResizeMode(3, qt.QHeaderView.Stretch)
    ShapetableWidgetLabel = qt.QLabel("   Needle shape received:")
    NeedleShapeGridLayout.addWidget(ShapetableWidgetLabel, 3, 0)
    NeedleShapeGridLayout.addWidget(self.ShapetableWidget, 3, 1)
    
    # Define empty Nodes 
    self.PlannedPathTransform = None
    self.XStageTransform = None
    #XStageMatrix = vtk.vtkMatrix4x4()
    #XStageMatrix.Identity()
    #self.XStageTransform.SetMatrixTransformToParent(XStageMatrix)
    
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
    
    #ReceivedNeedleShape = slicer.vtkMRMLLinearTransformNode()
    #ReceivedNeedleShape.SetName("currentshape")
    #slicer.mrmlScene.AddNode(ReceivedNeedleShape)
    
    XStageTransform = slicer.vtkMRMLLinearTransformNode()
    XStageTransform.SetName("XStageTransform")
    slicer.mrmlScene.AddNode(XStageTransform)
    
    # Add observers on the message type nodes
    #ReceivedStringMsg.AddObserver(slicer.vtkMRMLTextNode.TextModifiedEvent, self.onTextNodeModified)
    
    #ReceivedNeedlePose.AddObserver(slicer.vtkMRMLTextNode.TextModifiedEvent, self.onNeedlePoseNodeModified)
    #ReceivedNeedleShape.AddObserver(slicer.vtkMRMLTransformNode.TransformModifiedEvent, self.onNeedleShapeNodeModified)
    
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
  
    
  # ----- Functions to set target and skin entry points ------	  
    
  def onTargetPointFiducialChanged(self):
    targetPointNode = self.targetPointNodeSelector.currentNode()
    if targetPointNode is not None:
    	if not targetPointNode.GetNumberOfControlPoints() == 0:
    		targetCoordinatesRAS = targetPointNode.GetNthControlPointPositionVector(0)
    		self.xTargetTextbox.setText(round(targetCoordinatesRAS[0],2))
    		self.yTargetTextbox.setText(round(targetCoordinatesRAS[1],2))
    		self.zTargetTextbox.setText(round(targetCoordinatesRAS[2],2))
    		
  def onSkinEntryPointFiducialChanged(self):
    skinEntryPointNode = self.skinEntryPointNodeSelector.currentNode()
    if skinEntryPointNode is not None:
      if not skinEntryPointNode.GetNumberOfControlPoints() == 0:
        if self.PlannedPathTransform:
          slicer.mrmlScene.RemoveNode(self.PlannedPathTransform)
          self.PlannedPathTransform = None
        self.PlannedPathTransform = slicer.vtkMRMLLinearTransformNode()
        self.PlannedPathTransform.SetName("PlannedPathTransform")
        slicer.mrmlScene.AddNode(self.PlannedPathTransform)
        #self.PlannedPathTransform.AddObserver(slicer.vtkMRMLTransformNode.TransformModifiedEvent, self.onPlannedPathTransformNodeModified)
        
        skinEntryCoordinatesRAS = skinEntryPointNode.GetNthControlPointPositionVector(0)
        self.xSkinEntryTextbox.setText(round(skinEntryCoordinatesRAS[0],2))
        self.ySkinEntryTextbox.setText(round(skinEntryCoordinatesRAS[1],2))
        self.zSkinEntryTextbox.setText(round(skinEntryCoordinatesRAS[2],2))
        skinEntryPointMatrix = vtk.vtkMatrix4x4()
        skinEntryPointMatrix.Identity()
        skinEntryPointMatrix.SetElement(0,3,skinEntryCoordinatesRAS[0])
        skinEntryPointMatrix.SetElement(1,3,skinEntryCoordinatesRAS[1])
        skinEntryPointMatrix.SetElement(2,3,skinEntryCoordinatesRAS[2])
        self.PlannedPathTransform.SetMatrixTransformToParent(skinEntryPointMatrix)
        if slicer.mrmlScene.GetFirstNodeByName("PlannedPathNeedle") is not None:
          PointerNodeToRemove = slicer.mrmlScene.GetFirstNodeByName("PlannedPathNeedle")
          slicer.mrmlScene.RemoveNode(PointerNodeToRemove)      
        self.AddPointerModel("PlannedPathNeedle")
        TransformNodeToDisplay = slicer.mrmlScene.GetFirstNodeByName("PlannedPathTransform")
        locatorModelNode = slicer.mrmlScene.GetFirstNodeByName("PlannedPathNeedle")
        locatorModelNode.SetAndObserveTransformNodeID(TransformNodeToDisplay.GetID())
        
        
        
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
    
  def onsendTargetPointButtonClicked(self):   
    print("Sending target point coordinates")
    targetMsgNode = slicer.mrmlScene.GetFirstNodeByName("TARGET_POINT")
    self.openIGTNode.RegisterOutgoingMRMLNode(targetMsgNode)
    self.openIGTNode.PushNode(targetMsgNode)
    
  def onsendSkinEntryPointButtonClicked(self):  
    print("Sending skin entry point coordinates")
    skinEntryMsgNode = slicer.mrmlScene.GetFirstNodeByName("SKIN_ENTRY_POINT")
    self.openIGTNode.RegisterOutgoingMRMLNode(skinEntryMsgNode)
    self.openIGTNode.PushNode(skinEntryMsgNode)  
   
   # ----- Functions to get needle pose feedback ------
     
  def onNeedlePoseNodeModified(poseNode, unusedArg2=None, unusedArg3=None):
    print("New needle pose was received")
    ReceivedNeedlePose = slicer.mrmlScene.GetFirstNodeByName("/stage/state/needle")
    concatenatePose = ReceivedNeedlePose.GetText()
    poseNode.poseTextbox.setText(concatenatePose)
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
    print("New needle shape was received")   
    #ReceivedNeedleShape = slicer.mrmlScene.GetFirstNodeByName("currentshape")
    
    test = slicer.mrmlScene.GetNodeByID("vtkMRMLLinearTransformNode5")
    testtransformMatrix = vtk.vtkMatrix4x4()
    test.GetMatrixTransformToParent(testtransformMatrix)
    print(testtransformMatrix)
    
    classname = test.GetClassName()
    print(classname)
    #shapeNode.AddPointerModel("ShapeNeedle")
    #TransformNodeToDisplay = test
    #shapelocatorModelNode = slicer.mrmlScene.GetFirstNodeByName("ShapeNeedle")
    #shapelocatorModelNode.SetAndObserveTransformNodeID(TransformNodeToDisplay.GetID())
      
    #transformMatrix = vtk.vtkMatrix4x4()
    #ReceivedNeedleShape.GetMatrixTransformToParent(transformMatrix)
    #print(transformMatrix)

    
    nbRows = shapeNode.ShapetableWidget.rowCount
    print("rbrows:")
    print(nbRows)
    nbColumns = shapeNode.ShapetableWidget.columnCount
    print("nbcolumns:")
    print(nbColumns)
    for i in range(nbRows):
      for j in range(nbColumns):
        val = testtransformMatrix.GetElement(i,j)
        val = round(val,2)
        print(val)
        shapeNode.ShapetableWidget.setItem(i , j, qt.QTableWidgetItem(str(val)))

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
    
    
    
    
    
    
    
    
    
    
    
  
