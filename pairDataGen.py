# Reference/tutorial to take a look at

# https://help.autodesk.com/view/MOBPRO/2019/ENU/?guid=__py_ref__tasks_2_assign_rigid_body_8py_example_html
# https://help.autodesk.com/view/MOBPRO/2019/ENU/?guid=__files_GUID_A1189AA0_3816_4350_B8F3_5383DEC25A33_htm
# https://mocappys.com/complete-guide-to-poses-in-motionbuilder/#07_Using_the_Pose_Controls
# https://github.com/eksod/Retargeter

from pyfbsdk import *
import os
import re

import random

def addJoint(jointMap, newJoint, parent, localTrans, adjChildList=[], adjTrans=False, jointOrder=list()):
    jointMap[newJoint] = (parent, localTrans)
    for k in jointMap:
        v = jointMap[k]
        if v[0] == parent and k!=newJoint:
            if k in adjChildList:
                if adjTrans:
                    adjTrans = [v[1][0]- localTrans[0], v[1][1]- localTrans[1], v[1][2]- localTrans[2]]
                    jointMap[k] = (newJoint, adjTrans)
                else:
                    jointMap[k] = (newJoint, v[1])
    
    if parent in jointOrder:
        n = jointOrder.index(parent)
        jointOrder.insert(n+1, newJoint)
    else:        
        jointOrder.insert(0, newJoint)
    
# Create a skeleton in a T-pose facing along the positive Z axis
def createSkeleton(pNamespace):
    # reference jointMap is copied from mixamo character tpose.
    jointMap = {
        #jointName,     (parentName,       translation  )
        #'Reference':    (None,           (  0,  0,  0)),
        'Hips':         ('None',    (  0,  0,  0)),    
        'LeftUpLeg':    ('Hips',         (8.20778, -6.75171, -1.59956)),
        'LeftLeg':      ('LeftUpLeg',    (0, -44.3705, 0.284643)),
        'LeftFoot':     ('LeftLeg',      (0, -44.4279, -2.98219)),     
        'LeftToeBase':  ('LeftFoot',     (0, -8.72867, 10.7106)),    
        'RightUpLeg':   ('Hips',         (-8.2078, -6.75166, -1.59956)),
        'RightLeg':     ('RightUpLeg',   (0, -44.3705, 0.286156)),
        'RightFoot':    ('RightLeg',     (0, -44.4277, -2.98379)),  
        'RightToeBase': ('RightFoot',    (0, -8.72867, 10.7106)),    
        'Spine':        ('Hips',         (0.      , 29.36511 , -2.242699)),
        'LeftArm':      ('Spine',        (15.16277, 10.421402, -4.86012)),
        'LeftForeArm':  ('LeftArm',      (27.8415, -8.94286e-05, 3.74589e-05)),
        'LeftHand':     ('LeftForeArm',  (28.3288, -1.74407e-05, 3.78045e-05)),    
        'RightArm':     ('Spine',        (-15.16277, 10.421402, -4.86012)),
        'RightForeArm': ('RightArm',     (-27.8415, -3.30792e-05, 1.16763e-05)),
        'RightHand':    ('RightForeArm', (-28.3288, 0, 5.5816e-05)),
        'Neck':         ('Spine',        (0, 16.6717, -2.51617)),
        'Head':         ('Neck',         (0, 9.61788, 1.68501)),
    }

    skeleton = {}

    headOrder, legOrder, armOrder= ['Hips', 'Spine', 'Neck', 'Head'], ['LeftUpLeg', 'LeftLeg', 'LeftFoot', 'LeftToeBase'], ['LeftArm', 'LeftForeArm', 'LeftHand']
    if random.random() < 0.3:
        addJoint(jointMap, "LeftHipJoint", "Hips", (0,0,0), ["LeftUpLeg"], False, legOrder)
        addJoint(jointMap, "RightHipJoint", "Hips", (0,0,0), ["RightUpLeg"], False)
    if random.random() < 0.3:
        addJoint(jointMap, "LowerBack", "Hips", (0,0,0), ["Spine"], False, headOrder)
    
    spineCntRange = range(1, 7) 
    weights=[1.5+1/x for x in spineCntRange]
    sum_weights= sum(weights)
    weights =[x/sum_weights for x in weights]
    spineRnd = random.random()
    sum_w = 0
    spineCnt = -1

    for i, w in enumerate(weights):
        sum_w += w
        if spineRnd < sum_w:
            spineCnt = i+1
            break
    if spineCnt == -1: spineCnt = 6
    
    lastSpine = 'Spine'
    if spineCnt > 1:
        origTrans = jointMap['Neck'][1]
        unitTrans = [origTrans[0]/spineCnt, origTrans[1]/spineCnt, origTrans[2]/spineCnt]
        jointMap['Spine'] = (jointMap['Spine'][0], unitTrans)
        for i in range(1, spineCnt):
            addJoint(jointMap, 'Spine'+str(i), lastSpine, unitTrans, ['LeftArm', 'RightArm', 'Neck'], False, headOrder)
            lastSpine= 'Spine'+str(i)

    if random.random() < 0.5: 
        addJoint(jointMap, 'LeftShoulder', lastSpine, (4.57045, 10.946, -2.62799), ['LeftArm'], True, armOrder)
        addJoint(jointMap, 'RightShoulder', lastSpine, (-4.56997, 10.9462, -2.62802), ['RightArm'], True)
        
    elif random.random() < 0.3 : 
        addJoint(jointMap, 'LeftShoulder', lastSpine, (0, 0, 0), ['LeftArm'], True, armOrder)
        addJoint(jointMap, 'RightShoulder', lastSpine, (0, 0, 0), ['RightArm'], True)
        
    if random.random() < 0.5:
        neckHeadCnt = 2
        
        origTrans = jointMap['Head'][1]
        unitTrans = [origTrans[0]/neckHeadCnt, origTrans[1]/neckHeadCnt, origTrans[2]/neckHeadCnt]
        
        addJoint(jointMap, 'Neck1', 'Neck', unitTrans, ['Head'], True, headOrder)
    
    #End Sites 
    #Head
    if random.random() < 0.5:
        headTrans = (0, 19, 1)
        addJoint(jointMap, 'Head_End', 'Head', headTrans, [], False, headOrder)

    # Leg
    if random.random() < 1:
        toeTrans = (0, 0, 6)
        addJoint(jointMap, 'LeftToeBase_End', 'LeftToeBase', toeTrans, [], False, legOrder)
        addJoint(jointMap, 'RightToeBase_End', 'RightToeBase', toeTrans, [], False)

    # Arm
    if random.random() < 1:
        addJoint(jointMap, 'LeftHand_End', 'LeftHand',  (6, 0, 0), [], False, armOrder)
        addJoint(jointMap, 'RightHand_End', 'RightHand',  (-6, 0, 0), [], False)


    scale = 0.6+ 0.8*random.random()
    link_scale = {}
    # Populate the skeleton with joints.
    other_root = findRoot()

    for jointName, (parentName, translation) in jointMap.iteritems():
        jointName_ = jointName if not skelExists(other_root, jointName) else jointName+"_"
                
        if jointName == 'Reference' or jointName == 'Hips':
            # If it is the reference node, create an FBModelRoot.
            joint = FBModelRoot(jointName_)
            
        else:
            # Otherwise, create an FBModelSkeleton.
            joint = FBModelSkeleton(jointName_)
        
        joint.LongName = pNamespace + ':' + joint.Name # Apply the specified namespace to each joint.
        joint.Color = FBColor(0.3, 0.8, 1)             # Cyan
        joint.Size = 150                               # Arbitrary size: big enough to see in viewport 
        joint.Show = True                              # Make the joint visible in the scene.
        
        # Add the joint to our skeleton.
        skeleton[jointName] = joint
        if "Right" not in jointName:
            ls = [0.6+ 0.8*random.random(), 0.6+ 0.8*random.random(), 0.6+ 0.8*random.random()]
            jointMap[jointName] = (parentName, (translation[0]*ls[0], translation[1]*ls[1], translation[2]*ls[2]))
            #print(jointName, ls)

            if "Left" in jointName:
                cor_right = "Right"+jointName[4:]
                r_parentName, r_translation = jointMap[cor_right]
                jointMap[cor_right] = (r_parentName, (r_translation[0]*ls[0], r_translation[1]*ls[1], r_translation[2]*ls[2]))
                
    # move Hips so that foot touches the ground (2cm above considering foot depth(?))
    joint_lower= 'LeftToeBase'
    lower_sum= 0
    footOffset = 2
    while joint_lower != 'Hips':
        translation = jointMap[joint_lower][1]
        lower_sum= lower_sum +translation[1]
        joint_lower= jointMap[joint_lower][0]
    hips_trans = jointMap['Hips'][1]
    jointMap['Hips'] = ('None', (hips_trans[0], -lower_sum+footOffset, hips_trans[2]))
        
    # Once all the joints have been created, apply the parent/child 
    # relationships to each of the skeleton's joints.
    jointOrder= headOrder+legOrder+armOrder
    def connectPlaceJoint(jointName):
        (parentName, translation)= jointMap[jointName]
        # Only assign a parent if it exists.
        if parentName != None and parentName in jointMap.keys():
            skeleton[jointName].Parent = skeleton[parentName] 

        # The translation should be set after the parent has been assigned.            
        skeleton[jointName].Translation = FBVector3d(translation)*scale

    for jointName in reversed(jointOrder):
        connectPlaceJoint(jointName)
        if "Left" in jointName:
            cor_right = "Right"+jointName[4:]
            connectPlaceJoint(cor_right)   
    
    return skeleton

# Characterize the skeleton and create a control rig.
def characterizeSkeleton(pCharacterName, pSkeleton, ctrlRig=False):
    # Create a new character.
    character = FBCharacter(pCharacterName)
    FBApplication().CurrentCharacter = character
    
    # Add each joint in our skeleton to the character.
    for jointName, joint in pSkeleton.iteritems():
        slot = character.PropertyList.Find(jointName + 'Link')
        # skip dummy nodes (e.g.: LeftHipJoint)
        if slot is not None:
            slot.append(joint)

    # Flag that the character has been characterized.
    character.SetCharacterizeOn(True)
    
    if ctrlRig:
        # Create a control rig using Forward and Inverse Kinematics,
        # as specified by the "True" parameter.
        character.CreateControlRig(True)
        
        # Set the control rig to active.
        character.ActiveInput = True
    
    return character

# This is the Motionbuilder mapping to use the same function. Edit this list or create your own.
mobuMap = {'Reference' : 'reference',
            'Hips':'Hips',
             'LeftUpLeg' : 'LeftUpLeg',
             'LeftLeg' : 'LeftLeg',
             'LeftFoot' : 'LeftFoot',
             'LeftToeBase': 'LeftToeBase',
             'RightUpLeg' : 'RightUpLeg',
             'RightLeg' : 'RightLeg',
             'RightFoot' : 'RightFoot',
             'RightToeBase': 'RightToeBase',
             'Spine' : 'Spine',
             'LeftArm' : 'LeftArm',
             'LeftForeArm' : 'LeftForeArm',
             'LeftHand' : 'LeftHand',
             'RightArm' : 'RightArm',
             'RightForeArm' : 'RightForeArm',
             'RightHand' : 'RightHand',
             'Head' : 'Head',
             'LeftShoulder' : 'LeftShoulder',
             'RightShoulder' : 'RightShoulder',
             'Neck' : 'Neck',
             'Spine1' : 'Spine1',
             'Spine2' : 'Spine2',
             'Spine3' : 'Spine3',
             'Spine4' : 'Spine4',
             'Spine5' : 'Spine5',
             'Spine6' : 'Spine6',
             'Spine7' : 'Spine7',
             'Spine8' : 'Spine8',
             'Spine9' : 'Spine9',
             'Neck1' : 'Neck1',
             'Neck2' : 'Neck2',
             'Neck3' : 'Neck3',
             'Neck4' : 'Neck4',
             'Neck5' : 'Neck5',
             'Neck6' : 'Neck6',
             'Neck7' : 'Neck7',
             'Neck8' : 'Neck8',
             'Neck9' : 'Neck9',
             'LeftHandThumb1' : 'LeftHandThumb1',
             'LeftHandThumb2' : 'LeftHandThumb2',
             'LeftHandThumb3' : 'LeftHandThumb3',
             'LeftHandIndex1' : 'LeftHandIndex1',
             'LeftHandIndex2' : 'LeftHandIndex2',
             'LeftHandIndex3' : 'LeftHandIndex3',
             'LeftHandMiddle1' : 'LeftHandMiddle1',
             'LeftHandMiddle2' : 'LeftHandMiddle2',
             'LeftHandMiddle3' : 'LeftHandMiddle3',
             'LeftHandRing1' : 'LeftHandRing1',
             'LeftHandRing2' : 'LeftHandRing2',
             'LeftHandRing3' : 'LeftHandRing3',
             'LeftHandPinky1' : 'LeftHandPinky1',
             'LeftHandPinky2' : 'LeftHandPinky2',
             'LeftHandPinky3' : 'LeftHandPinky3',
             'RightHandThumb1' : 'RightHandThumb1',
             'RightHandThumb2' : 'RightHandThumb2',
             'RightHandThumb3' : 'RightHandThumb3',
             'RightHandIndex1' : 'RightHandIndex1',
             'RightHandIndex2' : 'RightHandIndex2',
             'RightHandIndex3' : 'RightHandIndex3',
             'RightHandMiddle1' : 'RightHandMiddle1',
             'RightHandMiddle2' : 'RightHandMiddle2',
             'RightHandMiddle3' : 'RightHandMiddle3',
             'RightHandRing1' : 'RightHandRing1',
             'RightHandRing2' : 'RightHandRing2',
             'RightHandRing3' : 'RightHandRing3',
             'RightHandPinky1' : 'RightHandPinky1',
             'RightHandPinky2' : 'RightHandPinky2',
             'RightHandPinky3' : 'RightHandPinky3',
             'LeftFootThumb1' : 'LeftFootThumb1',
             'LeftFootThumb2' : 'LeftFootThumb2',
             'LeftFootThumb3' : 'LeftFootThumb3',
             'LeftFootIndex1' : 'LeftFootIndex1',
             'LeftFootIndex2' : 'LeftFootIndex2',
             'LeftFootIndex3' : 'LeftFootIndex3',
             'LeftFootMiddle1' : 'LeftFootMiddle1',
             'LeftFootMiddle2' : 'LeftFootMiddle2',
             'LeftFootMiddle3' : 'LeftFootMiddle3',
             'LeftFootRing1' : 'LeftFootRing1',
             'LeftFootRing2' : 'LeftFootRing2',
             'LeftFootRing3' : 'LeftFootRing3',
             'LeftFootPinky1' : 'LeftFootPinky1',
             'LeftFootPinky2' : 'LeftFootPinky2',
             'LeftFootPinky3' : 'LeftFootPinky3',
             'RightFootThumb1' : 'RightFootThumb1',
             'RightFootThumb2' : 'RightFootThumb2',
             'RightFootThumb3' : 'RightFootThumb3',
             'RightFootIndex1' : 'RightFootIndex1',
             'RightFootIndex2' : 'RightFootIndex2',
             'RightFootIndex3' : 'RightFootIndex3',
             'RightFootMiddle1' : 'RightFootMiddle1',
             'RightFootMiddle2' : 'RightFootMiddle2',
             'RightFootMiddle3' : 'RightFootMiddle3',
             'RightFootRing1' : 'RightFootRing1',
             'RightFootRing2' : 'RightFootRing2',
             'RightFootRing3' : 'RightFootRing3',
             'RightFootPinky1' : 'RightFootPinky1',
             'RightFootPinky2' : 'RightFootPinky2',
             'RightFootPinky3' : 'RightFootPinky3',
             'LeftUpLegRoll' : 'LeftUpLegRoll',
             'LeftLegRoll' : 'LeftLegRoll',
             'RightUpLegRoll' : 'RightUpLegRoll',
             'RightLegRoll' : 'RightLegRoll',
             'LeftArmRoll' : 'LeftArmRoll',
             'LeftForeArmRoll' : 'LeftForeArmRoll',
             'RightArmRoll' : 'RightArmRoll',
             'RightForeArmRoll' : 'RightForeArmRoll' }

def addJointToCharacter ( characterObject, slot, jointName ):    
    myJoint = FBFindModelByLabelName(jointName)
    if myJoint:
        proplist = characterObject.PropertyList.Find(slot + "Link")    
        proplist.append (myJoint)

def CharacterizeBiped(rootname, useBipedPrefixNamingScheme, nameprefix, boneMap, models):
  
    system = FBSystem()
    app = FBApplication()    
    
    longname = models.LongName
    namespaceindex = longname.rfind(":")
    if namespaceindex != -1:
        namespace = longname[0:namespaceindex+1] 
        name = longname[namespaceindex + 1:]
    else:
        namespace = ""
        name = longname

    myBiped = FBCharacter("mycharacter")
    app.CurrentCharacter = myBiped
    
    # If in Biped mode, extract the character prefix name
    if useBipedPrefixNamingScheme:
        splitname = name.split()
        nameprefix = splitname[0] + " "
        # Override the rootname so it is the character orefix name            
        rootname = splitname[0]
        myBiped.LongName = namespace + rootname
    else:
        myBiped.LongName = namespace + nameprefix + rootname
   
                
    # Create a FBProgress object and set default values for the caption and text.    
    fbp = FBProgress()
    fbp.Caption = ""
    fbp.Text = " ----------------------------------- Creating Biped character"
    progress = 0.0
    progresssteps = len(boneMap)

    # assign Biped to Character Mapping.
    for pslot, pjointName in boneMap.iteritems():
        if not pjointName:
            addJointToCharacter(myBiped, pslot, namespace + rootname)
        else:
            addJointToCharacter(myBiped, pslot, namespace + nameprefix + pjointName)
        progress += 1
        val = progress / len(boneMap)  * 100
        fbp.Percent = int(val)
                
    switchOn = myBiped.SetCharacterizeOn( True )    
    # print "Character mapping created for " + (myBiped.LongName)
        
    # We must call FBDelete when the FBProgress object is no longer needed.
    fbp.FBDelete()
    return myBiped

def createControlRig(character, activeInput=True):
    # Create a control rig using Forward and Inverse Kinematics,
    # as specified by the "True" parameter.
    character.CreateControlRig(True)
    
    # Set the control rig to active.
    if activeInput:
        character.ActiveInput = True


def plotAnim(char, animChar):
    """
    Receives two characters, sets the input of the first character to the second
    and plot. Return ploted character.
    """
    #if char.GetCharacterize:
    #    switchOn = char.SetCharacterizeOn(True)

    plotoBla = FBPlotOptions()
    plotoBla.ConstantKeyReducerKeepOneKey = True
    plotoBla.PlotAllTakes = True
    plotoBla.PlotOnFrame = True
    plotoBla.PlotPeriod = FBTime( 0, 0, 0, 1 )
    plotoBla.PlotTranslationOnRootOnly = True
    plotoBla.PreciseTimeDiscontinuities = True
    #plotoBla.RotationFilterToApply = FBRotationFilter.kFBRotationFilterGimbleKiller
    plotoBla.UseConstantKeyReducer = False
    plotoBla.ConstantKeyReducerKeepOneKey  = True
    char.InputCharacter = animChar
    char.InputType = FBCharacterInputType.kFBCharacterInputCharacter
    char.ActiveInput = True
    if (not char.PlotAnimation(FBCharacterPlotWhere.kFBCharacterPlotOnSkeleton, plotoBla)):
        FBMessageBox( "Something went wrong", "Plot animation returned false, cannot continue", "OK", None, None )
        return False

    return char    



def SwitchTake( pTakeName ):
    iDestName= pTakeName
    for iTake in FBSystem().Scene.Takes:
        if iTake.Name == iDestName:
            FBSystem().CurrentTake = iTake

def findRoot():
    for child in FBSystem().Scene.RootModel.Children:
        if "Hips" in child.Name:
            return child
    return None

def skelExists(root, name):
    if root == None: return False
    if root.Name == name: return True
    for child in root.Children:
        if skelExists(child, name):
            return True
    return False
###############################################################
# Main.                                                       #
###############################################################
rootDir = "C:/Users/sunmin/Documents/GitHub/MBScript/"
bvhFileDir = os.path.join(rootDir, "bvh/")
dirList = [os.path.join(bvhFileDir, subDir) for subDir in os.listdir(bvhFileDir) if os.path.isdir(os.path.join(bvhFileDir, subDir))]
fileList = []

## filter short clips
for subDir in dirList:
    allList = [os.path.join(subDir, f) for f in os.listdir(subDir) if f.endswith('.bvh')]
    fileList += allList
    #dirFileList.append(allList)

system = FBSystem()
app = FBApplication()    


newFileDir = os.path.join(rootDir, "result/")
if not os.path.exists(newFileDir):
    os.makedirs(newFileDir)

logPath = os.path.join(newFileDir, "result_log.txt")
logFile = open(logPath, 'w')

for idx, filePath in enumerate(fileList):
    print(idx, filePath)
    fileName = filePath.split(os.sep)[-1]
    fileType = filePath.split(os.sep)[-2]

    logStr = str(idx)+','+fileType+','+fileName+"\n"
    logFile.write(logStr)
    userRoot = "Hips" 
    boneMap = mobuMap
    bipedPrefixNamingScheme = False
    prefix = ""
    
    app.FileNew()
    success = app.FileImport(filePath, True)
    poseOptions = FBCharacterPoseOptions()
    poseOptions.mCharacterPoseKeyingMode = FBCharacterPoseKeyingMode.kFBCharacterPoseKeyingModeFullBody

    animRoot = findRoot()
    animChar = CharacterizeBiped(userRoot, bipedPrefixNamingScheme, prefix, boneMap, animRoot)
    animChar.SelectModels(True, True, True, False)
    lPlayer = FBPlayerControl()
    lPlayer.Goto(FBTime(0, 0, 0, 0))
    createControlRig(animChar, False)

    # first key all frame for animation bvh to prevent unwanted interpolation between frames
    lEndTime = system.CurrentTake.LocalTimeSpan.GetStop()
    lEndFrame = system.CurrentTake.LocalTimeSpan.GetStop().GetFrame()
    lStartFrameTime = system.CurrentTake.LocalTimeSpan.GetStart()
    lStartFrame = system.CurrentTake.LocalTimeSpan.GetStart().GetFrame()

    lRange = min(int(lEndFrame)+1, 50)
    lPlayer = FBPlayerControl()

    for i in range(lRange):
        lPlayer.Goto(FBTime(0, 0, 0, i))
        system.Scene.Evaluate() 
        lPlayer.Key()
        FBSystem().Scene.Evaluate()
        
    # Create a new skeleton.
    characterName = 'SkelTestCharacter'
    skeleton = createSkeleton(characterName)
    character = characterizeSkeleton(characterName, skeleton, False)

    plotAnim(character, animChar)
    animChar.SelectModels(False, True, True, True)
    character.SelectModels(True, True, True, True)

    # setup save options (for some reason, they were not working outside this loop...)
    sOptions = FBFbxOptions(False) # false = save options
    sOptions.SaveCharacter = True
    sOptions.SaveControlSet = False
    sOptions.SaveCharacterExtension = False
    sOptions.ShowFileDialog = False
    sOptions.ShowOptionsDialog = False

    fileName = str(idx)+'.bvh'
    newPath = os.path.join(newFileDir, fileName)
    FBApplication().FileExport(newPath)

logFile.close()

