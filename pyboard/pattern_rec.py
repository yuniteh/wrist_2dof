#The class replicates a pattern recognition system capable of performing LDA classification 
#from time domain features
#
#Authors: Levi Hargrove and Kevin Brenner
#Jan 20, 2020

import pyb, utime, ubinascii, micropython, os,gc,time, config
import ulab as np
from ulab import linalg

# Code to be run in micropython
def timeit(f, *args, **kwargs):
    func_name = str(f).split(' ')[1]
    def new_func(*args, **kwargs):
        t = utime.ticks_us()
        result = f(*args, **kwargs)
        #print('execution time: ', utime.ticks_diff(utime.ticks_us(), t), ' us')
        return result
    return new_func

class PatternRec:
  # Initialization function. Set up variables, register callbacks, etc.
  def __init__(self,save_helper):
    config_inst = config.Config()
    # List of available EMG channels (change if you have more or less channels)
    self._emg_chan_list = config_inst.emg_chan_list
    # Calculate the total number of features as 4*number of emg channels
    self._total_num_feats = config_inst.total_num_feats
    # Our classifier will have 10 possible decisions:
    # No Movement,Flexion,Extension,Pronation,Supination,Hand Open,Grip0,Grip1,
    # Grip2,Grip3
    self._num_classes = config_inst.num_classes 
    # Create a local version of the save_helper class instance
    self._save_helper = save_helper
    # Map the different classes to their respective index values (these are
    # taken from the file DOFDEF.dof [I believe])
    # No Movement, Flexion, Extension, Pronation, Supination, Hand Open, Power,
    # Key, Chuck, Pinch, Point, Rock, Trigger, HandShake, ChuckOk, Robustness
    self._class_mapper = [1,12,13,11,10,16,19,17,18,20,22,24,25,26,27,0]
    # Map the order of the various grip configurations
    self._grip_dict = {0:"Power",1:"Key",2:"Chuck",3:"Pinch",4:"Point",5:"Rock",6:"Trigger",7:"HandShake",8:"ChuckOk"}
    # Create a mapping for the differnt grips and there respective speed values
    # -1 (or 255) means that the grip of question is not activated
    self._grip_mapper = {"Grip0":-1,"Grip1":-1,"Grip2":-1,"Grip3":-1}
    
    # Switch the directory to the SD card
    os.chdir('/sd')
    # Determine which terminal device was used last and establish number of
    # classes along with the class mapper with this information
    f = open('td_type.txt','r')
    self._td_type = f.read()
    f.close()
    self.initializeClassifierVariables()        
    os.chdir('/sd')

  # Handles the initialization of local pattern recognition variables such as
  # means, covariances, weights, and cg's
  #
  # Inputs:
  #   N/A
  #
  # Returns:
  #   N/A
  def initializeClassifierVariables(self):
    # Check to see if we have existing means, covariances, number of samples,
    # and lda models
    # If not, then create them. 
    tmp = os.listdir()
    controller_folder_exists = False
    for x in tmp:
      if 'ControllerData' in x:
        controller_folder_exists = True
        os.chdir('/sd/ControllerData')
    print(controller_folder_exists)
    # If there is no controller data foldre somehow (shouldn't ever be the
    # case), then create one.
    # Or, if there is such a folder, but it is missing most of the relevant
    # variables (also shouldn't be possible), create empty entries for all of
    # the classifier variables.
    if controller_folder_exists == False or len(os.listdir()) < 2:
      if controller_folder_exists == False:
        os.mkdir('ControllerData')
      os.chdir('/sd/ControllerData')
      empty_cov = np.zeros((self._total_num_feats,self._total_num_feats))
      empty_mean = np.zeros((1,self._total_num_feats))
      # Loop through the number of possible classes and create empty entries
      # for all of the classifier variables
      for x in range(self._num_classes):
        self._save_helper.writeMN_COV(empty_cov,0,('COV'+str(x)+'.DAP'))
        self._save_helper.writeMN_COV(empty_mean,0,('MN'+str(x)+'.DAP'))
      self._save_helper.writeMN_COV(np.zeros((self._total_num_feats,self._total_num_feats)),0,('INV_COV.DAP'))
      self._save_helper.writeFloatMat(np.zeros((self._total_num_feats,self._num_classes), dtype = np.float),'WG.DAP')
      self._save_helper.writeFloatMat(np.zeros((1,self._num_classes), dtype = np.float),'CG.DAP')
      self._save_helper.writeFloatMat(np.zeros((1,self._num_classes), dtype = np.float),'Class_List.DAP')
      self._save_helper.writeFloatMat(np.array([[0.0,0.0]]),'NM_Thresh.txt')
      self._save_helper.writeFloatMat(np.array([[-1,-1,-1,-1,-1,-1,-1,-1,-1,-1]]),'Grip_Data.DAP')
    # Otherwise, just switch the directory to the controller data directory
    else:
      os.chdir('/sd/ControllerData')

    work_dir = os.getcwd()
    # Initialize empty or rest values for each of the relevant variables
    # Feature matrix
    self._feats = np.zeros((1,self._total_num_feats))
    # Flag to track if we are currently calibrating (not prepping)
    self._calibrating_flag = False
    # Tracking which class is being collected (-1 is prepping)
    self._calibration_class = -1
    # EMG data
    self._mav_bytes = bytearray(17)
    # Address for the EMG data to be sent to the PAT App
    self._mav_bytes[0] = 0x04
    # Pooled covariance matrix
    self._pooled_cov = np.zeros((self._total_num_feats,self._total_num_feats))
    # Inverse pooled covariance matrix
    self._inv_pooled_cov = np.zeros((self._total_num_feats,self._total_num_feats))
    # Old zero mean feats matrix
    self._zero_mean_feats_old = np.zeros((1,self._total_num_feats))
    # New zero mean feats matrix
    self._zero_mean_feats_new = np.zeros((1,self._total_num_feats))
    # Dot product of old feats with new feats
    self._point_cov = np.zeros((self._total_num_feats,self._total_num_feats))
    # Historical buffer of previous feature matrices
    self._feat_hist_buffer = np.zeros((10,self._total_num_feats))
    # Keeping track of how many frames of feature data are tracked
    self._feat_hist_count = 0
    # Covariance matrix
    self._cov = np.zeros((self._total_num_feats,self._total_num_feats))
    # Indices to keep track of which column of the covariance matrix we want to
    # update during the current calibration sequence
    self._cov_index = -1
    # Keeping time history of the previous covariance index as well
    self._prev_cov_index = -1
    # Means
    self._means = []
    # Number of instances
    self._N = []
    # No movement threshold
    self._NM_threshold = 0.0
    # Average no movement threshold
    self._avg_NM_threshold = 0.0
    # Number of no movement threshold examples
    self._num_NM_threshold_examples = 0.0
    # A flag to determine if we want to use raw or filtered data during
    # training
    self._use_raw_data = False
    # A flag to keep track if you are in the process of training
    self._is_training = False
    # The gain to be multiplied to the no movement threshold
    self._thresh_gain = 1.1
    # List of which classes have been trained for the classifier.
    # This will be empty when the classifier is reset.
    self._class_list = []
    # Information related to which grips are available and their respective
    # speed settings
    self._class_grip_data = []

    # If they don't exist (which they should), create subdirectories for each
    # type of terminal device (Ottobock vs. Psyonic).
    # These folders will be where we save classifier models for each terminal
    # device.
    tmp = os.listdir()
    sub_folder_exists = False
    for x in tmp:
      if 'ControllerDataSubFolder' in x:
        sub_folder_exists = True
    if sub_folder_exists == False:
      os.mkdir('ControllerDataSubFolder')
      print("made sub folder directory")
    os.chdir('/sd/ControllerData/ControllerDataSubFolder')
    tmp = os.listdir()
    ottobock_folder_exists = False
    for x in tmp:
      if 'Ottobock' in x:
        ottobock_folder_exists = True
    if ottobock_folder_exists == False:
      os.mkdir('Ottobock')
    psyonic_folder_exists = False
    for x in tmp:
      if 'Psyonic' in x:
        psyonic_folder_exists = True
    if psyonic_folder_exists == False:
      os.mkdir('Psyonic')
    os.chdir('/sd/ControllerData')

    # Load in local classifier's means and number of examples
    # For now, we are not reading in the entire covariance matrix because it
    # is causing memory issues, so instead we will be reading in individual
    # columns of the larger covariance matrix when needed
    for x in range(self._num_classes):
      #gc.collect()
      #[tmpCOV,tmp_N] = self._save_helper.readMN_COV(('COV'+str(x)+'.DAP'))
      #gc.collect()
      [tmp_MN,tmp_N] = self._save_helper.readMN_COV(('MN'+str(x)+'.DAP'))
      #self._cov.append(tmpCOV)
      self._means.append(tmp_MN)
      self._N.append(tmp_N[0])

    # Read in the local classifier's inverse covariance matrix as well as the
    # weights and cg's
    [tmp_inv_cov,tmp_N] = self._save_helper.readMN_COV(('INV_COV.DAP'))
    self._inv_pooled_cov = tmp_inv_cov
    self._wg = self._save_helper.readFloatMat('WG.DAP')
    self._cg = self._save_helper.readFloatMat('CG.DAP')

    # Read in the local classifier's no movement threshold
    NM_thresh_data = self._save_helper.readFloatMat('NM_Thresh.txt')
    self._avg_NM_threshold = NM_thresh_data[0][0]
    self._num_NM_threshold_examples = NM_thresh_data[0][1]

    # Read in the local classifier's class list
    class_list_data = self._save_helper.readFloatMat('Class_List.DAP')
    for x in range(self._num_classes):
      self._class_list.append(int(class_list_data[0,x]))
    print("Class List: ",self._class_list)

    # Read in the local classifier's grip data
    grip_data = self._save_helper.readFloatMat('Grip_Data.DAP')
    for x in range(10):
      self._class_grip_data.append(int(grip_data[0,x]))

    # If the variable class_list is not empty, then there is an available
    # classifier; otherwise, there isn't
    if self._class_list != [0]*self._num_classes:
      self._classifier_available = True
      #print("classifier is available : ",self._class_list)
    else:
      self._classifier_available = False
      #print("classifier not available : ",self._class_list)

    # Initialize the alpha variable, which will be used in the update means and
    # covariances function
    self._alpha = []
    for x in range(self._num_classes):
      self._alpha.append(0)

    os.chdir(work_dir)

  # Resetting the controller means wiping the local classifier model of all of
  # its variable values (means, covariances, weights, cg's, etc.)
  #
  # Inputs:
  #   N/A
  #
  # Returns:
  #   N/A
  def resetController(self):
    # Basically just go to the local controller data folder and zero out all
    # matrices, reset all variables to their initial values, and turn off all
    # flags
    work_dir = os.getcwd()
    os.chdir('/sd/ControllerData')
    for x in range(self._num_classes):
      self._save_helper.writeMN_COV(np.zeros((self._total_num_feats,self._total_num_feats)),0,('COV'+str(x)+'.DAP'))
      self._save_helper.writeMN_COV(np.zeros((1,self._total_num_feats)),0,('MN'+str(x)+'.DAP'))
      self._N[x] = 0
    self._NM_threshold = 0.0
    self._avg_NM_threshold = 0.0
    self._num_NM_threshold_examples = 0.0
    self._save_helper.writeMN_COV(np.zeros((self._total_num_feats,self._total_num_feats)),0,('INV_COV.DAP'))
    self._save_helper.writeFloatMat(np.array([[0.0,0.0]]),'NM_Thresh.txt')
    self._save_helper.writeFloatMat(np.zeros((self._total_num_feats,self._num_classes), dtype = np.float),'WG.DAP')
    self._save_helper.writeFloatMat(np.zeros((1,self._num_classes), dtype = np.float),'CG.DAP')
    self._save_helper.writeFloatMat(np.zeros((1, self._num_classes), dtype=np.float), 'Class_List.DAP')
    self._wg = self._save_helper.readFloatMat('WG.DAP')
    self._cg = self._save_helper.readFloatMat('CG.DAP')
    class_list_data = self._save_helper.readFloatMat('Class_List.DAP')
    self._class_list = []
    self._cov = np.zeros((self._total_num_feats,self._total_num_feats))
    self._cov_index = -1
    self._means = []
    for x in range(self._num_classes):
      #[tmpCOV, tmp_N] = self._save_helper.readMN_COV(('COV' + str(x) + '.DAP'))
      [tmp_MN, tmp_N] = self._save_helper.readMN_COV(('MN' + str(x) + '.DAP'))
      #self._cov.append(tmpCOV)
      self._means.append(tmp_MN)
    self._inv_pooled_cov = np.zeros((self._total_num_feats,self._total_num_feats))
    for x in range(self._num_classes):
      self._class_list.append(int(class_list_data[0,x]))
    self._classifier_available = False
    os.chdir(work_dir)

  # There might be an instance where you want to reset specific motions while
  # maintaining others (i.e. rotation is working fine, but flexion isn't)
  #
  # Inputs:
  #   classes_to_reset:                     a list of classes to be reset in
  #                                         the classifier
  #
  # Returns:
  #   N/A
  def resetSpecificClasses(self,classes_to_reset):
    # Loop through the indices of these specific classes, and zero out the
    # means and covariances.
    work_dir = os.getcwd()
    os.chdir('/sd/ControllerData')
    for y in range(len(classes_to_reset)):
      x = classes_to_reset[y]
      self._means[x] = np.zeros((1,self._total_num_feats))
      self._save_helper.writeMN_COV(self._means[x],0,('MN'+str(x)+'.DAP'))
      self._save_helper.writeMN_COV(np.zeros((self._total_num_feats,self._total_num_feats)),0,('COV'+str(x)+'.DAP'))
      self._N[x] = 0
      self._class_list[x] = 0

    # Then recalculate the inverse covariance matrix and create a new
    # classifier model based on the new pooled covariance matrix.
    # You will get new weights and cg's out of the makeLDAClassifier()
    # function that will refelct the absence of specific classes.
    self._cov_index = -1
    self.calculateInverseCovariance()
    self.makeLDAClassifier()
    
    # Double check to make sure that you didn't selectively reset all of the
    # classes.
    # Read in the local classifier's class list.
    self._class_list = []
    class_list_data = self._save_helper.readFloatMat('Class_List.DAP')
    for x in range(self._num_classes):
      self._class_list.append(int(class_list_data[0,x]))
    #print("Class List: ",self._class_list)
    # If the variable class_list is not empty, then there is an available
    # classifier; otherwise, there isn't
    if self._class_list != [0]*self._num_classes:
      self._classifier_available = True
      #print("classifier is available : ",self._class_list)
    else:
      self._classifier_available = False
      #print("classifier not available : ",self._class_list)

    os.chdir(work_dir)

  # Updating the mean and covariance matrices for the local classifier during
  # a calibration session
  #
  # Inputs:
  #   class_index:                          an index that corresponds to the
  #                                         class index to be updated
  #
  # Returns:
  #   N/A
  def updateMeanAndCov(self,class_index):
    # Flag to determine if you should update the means and covariances
    update_mean_and_cov = True

    # If you are collecting no movement data, update the no movement threshold
    if class_index == 0:
      class_index_relabeled = class_index
      # Update the no movement threshold
      self._NM_threshold = np.mean(self._feats[0:8])
      self._num_NM_threshold_examples = self._num_NM_threshold_examples + 1.0
      self._avg_NM_threshold = ((self._avg_NM_threshold*(self._num_NM_threshold_examples-1))+self._NM_threshold)/self._num_NM_threshold_examples
      self._NM_threshold = np.mean(self._feats[0:8])
      self._num_NM_threshold_examples = self._num_NM_threshold_examples + 1.0
      self._avg_NM_threshold = ((self._avg_NM_threshold*(self._num_NM_threshold_examples-1))+self._NM_threshold)/self._num_NM_threshold_examples
    # If you are collecting data for the robustness class, just update the
    # threshold variable, but don't update any of the means and covariances
    elif class_index == 10:
      class_index_relabeled = 0
    # If you are training any other class, then only update the means and
    # covariances if the feature set is above the no movement threshold times
    # the threshold gain (that way we don't classify data that should have
    # been no movement as something else)
    else:
      class_index_relabeled = class_index
      if not (np.mean(self._feats[0:8]) >= self._thresh_gain*self._NM_threshold):
        update_mean_and_cov == False
    if update_mean_and_cov:
      # Updating the means, covariances, and class list
      self._alpha[class_index_relabeled] = self._N[class_index_relabeled]/(self._N[class_index_relabeled]+1)
      self._N[class_index_relabeled] = self._N[class_index_relabeled]+1
      # De-mean based on old mean value
      self._zero_mean_feats_old = self._feats-self._means[class_index_relabeled]
      # Update the mean vector
      self._means[class_index_relabeled] = self._means[class_index_relabeled] * self._alpha[class_index_relabeled] + self._feats * (1-self._alpha[class_index_relabeled])
      # De-mean based on the updated mean value
      self._zero_mean_feats_new = self._feats-self._means[class_index_relabeled]
      self._point_cov = linalg.dot(self._zero_mean_feats_old.transpose(),self._zero_mean_feats_new)
      self._cov = self._cov * self._alpha[class_index_relabeled] + self._point_cov * (1-self._alpha[class_index_relabeled])
      self._class_list[class_index_relabeled] = 1
   
  # Calculate a desired class out based upon the new frme of EMG data
  #
  # Inputs:
  #   N/A
  #
  # Returns:
  #   N/A
  def performClassification(self):
    # Create a copy of the feature matrix
    revised_feats = self._feats.copy()
    # If an emg channel has been toggled off, don't use it in making the
    # calculation of the desired class out
    for y in range(8):
      if y not in self._emg_chan_list:
        # Swap out 4 if change the number of features per EMG channel
        revised_feats[0,y * 4:(y + 1) * 4] = 0
    # Calculate the dot product of the feature matrix and the weights and then
    # add the cg's to this list
    a = linalg.dot(revised_feats,self._wg) + self._cg
    # Determine the argmax of this list to determine which class most closesly
    # aligns with the new set of feature data
    b = np.argmax(a[0])
    # If the entry at the index that is determined as the class out doesn't
    # actually have a trained classifier for it, then just set the output to be
    # no movement (this shouldn't occur?) 
    if self._class_list[b] != 1:
      b = 0

    # If the terminal device is Ottobock, then just map it directly based on
    # the index
    if self._td_type == "Ottobock":
      return self._class_mapper[b]
    # If the terminal device is Psyonic, and the output is one of the grip
    # types, then map it using the dictionary of different grip types so that
    # the output is the actual grip itself and not just something like "Grip0"
    elif self._td_type == "Psyonic":
      if b < 6:
        return self._class_mapper[b]
      else:
        if b == 6:
          return self._class_mapper[self._grip_mapper["Grip0"]+6]
        elif b == 7:
          return self._class_mapper[self._grip_mapper["Grip1"]+6]
        elif b == 8:
          return self._class_mapper[self._grip_mapper["Grip2"]+6]
        elif b == 9:
          return self._class_mapper[self._grip_mapper["Grip3"]+6]
  
  # Save the local classifier to a specific folder name
  #
  # Inputs:
  #   folder_name:                          the name of the folder where the
  #                                         variables should be saved
  #
  # Returns:
  #   N/A
  def saveController(self, folder_name):
    # Initialize the variable "valid folder" to be true.
    # This variable will become False if the folder_name isn't valid.
    valid_folder = True
    work_dir = os.getcwd()

    # Switch the location of where the folder will be saved to the appropriate
    # terminal device-specific subdirectory
    final_path = '/sd/ControllerData/ControllerDataSubFolder/' + self._td_type

    # Loop through that folder path and check to see if the folder name of
    # interest already exists there
    os.chdir(final_path)
    tmp = os.listdir()
    for x in tmp:
      if folder_name in x:
        valid_folder = False

    # If the folder is an acceptable name, then create a new directory for it,
    # and change directories to that path
    if valid_folder:
      folder_path = final_path + "/" + folder_name
      os.mkdir(folder_path)
      os.chdir(folder_path)

      # Write the classifier variables to this folder location
      for x in range(self._num_classes):
        #self._save_helper.writeMN_COV(self._cov[x],self._N[x],('COV'+str(x)+'.DAP'))
        self._save_helper.writeMN_COV(self._means[x],self._N[x],('MN'+str(x)+'.DAP'))
      self._save_helper.writeMN_COV(self._inv_pooled_cov,self._N[x],('INV_COV.DAP'))
      self._save_helper.writeFloatMat(self._wg,'WG.DAP')
      self._save_helper.writeFloatMat(self._cg,'CG.DAP')

      self._save_helper.writeFloatMat(np.array([[self._avg_NM_threshold,self._num_NM_threshold_examples]]),'NM_Thresh.txt')
      self._save_helper.writeFloatMat(np.array([self._class_list]), 'Class_List.DAP')
      self._save_helper.writeFloatMat(np.array([self._class_grip_data]), 'Grip_Data.DAP')

      # Read in the covariance matrix from the local classifier, column by
      # column, and write each column to the saved folder location, column by
      # column as well
      for x in range(self._num_classes):
        gc.collect()
        os.chdir('/sd/ControllerData')
        [tmpCOV, tmp_N] = self._save_helper.readMN_COV(('COV' + str(x) + '.DAP'))
        cov = tmpCOV
        gc.collect()
        os.chdir(folder_path)
        self._save_helper.writeMN_COV(cov,self._N[x],('COV'+str(x)+'.DAP'))

    os.chdir(work_dir)

    return valid_folder

  # Calculating the inverse covariance matrix based on the covariance matrix
  #
  # Inputs:
  #   N/A
  #
  # Returns:
  #   N/A
  def calculateInverseCovariance(self):
    os.chdir('/sd/ControllerData')
    # Creating an empty pooled covariance matrix
    self._pooled_cov = np.zeros((self._total_num_feats,self._total_num_feats),dtype=np.float)
    # Looping through each class
    for x in range(self._num_classes):
      # If a class has been trained, read in the covariance matrix for that
      # class and add it to the pooled covariance matrix
      if self._class_list[x] == 1:
        [tmpCOV, tmp_N] = self._save_helper.readMN_COV(('COV' + str(x) + '.DAP'))
        cov = tmpCOV
        self._pooled_cov = self._pooled_cov + cov
    # Divide the pooled covariance matrix by the total number of classes to
    # get the average pooled covariance matrix
    self._pooled_cov = self._pooled_cov/(sum(self._class_list))
    try:
      gc.collect()
      # Invert the average pooled covariance matrix to get the inverse pooled
      # covariance matrix
      self._inv_pooled_cov = linalg.inv(self._pooled_cov)
    except Exception as e:
      print(e)

  # Calculate the weights and offsets for the classifier and write these
  # variables to file
  #
  # Inputs:
  #   N/A
  #
  # Returns:
  #   N/A
  def makeLDAClassifier(self):
    # Create a copy of the means matrix and then set the entries to zero that
    # correspond to the EMG channels that we wish to toggle off
    revised_means = self._means.copy()
    for x in range(self._num_classes):
      class_revised_means = revised_means[x]
      class_means = class_revised_means.copy()
      for y in range(8):
        if y not in self._emg_chan_list:
          print("y: ", y)
          # Swap out 4 if change the number of features per EMG channel
          class_means[0,y * 4:(y + 1) * 4] = 0
        else:
          pass
      revised_means[x] = class_means
    # Create a copy of the inverse pooled covariance matrix and then zero out
    # the appropriate rows/columns that correspond to the EMG channels we don't
    # want to include
    revised_inv_pooled_cov = self._inv_pooled_cov.copy()
    for x in range(8):
      if x not in self._emg_chan_list:
        print("x: ",x)
        # Swap out 4 if change the number of features per EMG channel
        revised_inv_pooled_cov[:, 4 * x:4 * (x + 1)] = 0
        revised_inv_pooled_cov[4 * x:4 * (x + 1), :] = 0
      else:
        pass

    # Initialize the weights and cg's matrices
    self._wg = np.zeros((self._total_num_feats,self._num_classes))
    self._cg = np.zeros((1,self._num_classes))

    # Loop through each class
    for x in range(self._num_classes):
      # If the class index has data collected, take the dot product of the
      # means and inverse pooled covariance matrices                                       
      if self._class_list[x] == 1:
        dum = linalg.dot(revised_means[x],revised_inv_pooled_cov.transpose())
        # Assign the dot product to the weights matrix in the column that
        # corresponds to the class of interest
        for i in range(self._total_num_feats):
          self._wg[i,x] = dum.transpose()[i]

        # Calculat the cg's in the following way, using the means and inverse
        # pooled covariance matrix
        tmp = linalg.dot(np.array(revised_means[x]),revised_inv_pooled_cov)
        tmp_2 = linalg.dot(revised_means[x],tmp.transpose())*(-0.5)
        self._cg[0,x] = tmp_2.transpose()

    # Since we have just created a classifier, set the flag to be true
    self._classifier_available = True

    work_dir = os.getcwd()
    os.chdir('/sd/ControllerData')

    # Log the local classifier data in the controller data folder.
    # Not writing the covariance matrix here since we are writing it to file,
    # column by column, during the calibration itself
    for x in range(self._num_classes):
      #self._save_helper.writeMN_COV(self._cov[x],self._N[x],('COV'+str(x)+'.DAP'))
      self._save_helper.writeMN_COV(self._means[x],self._N[x],('MN'+str(x)+'.DAP'))
    self._save_helper.writeMN_COV(self._inv_pooled_cov,self._N[x],('INV_COV.DAP'))
    self._save_helper.writeFloatMat(self._wg,'WG.DAP')
    self._save_helper.writeFloatMat(self._cg,'CG.DAP')

    self._save_helper.writeFloatMat(np.array([[self._avg_NM_threshold,self._num_NM_threshold_examples]]),'NM_Thresh.txt')
    self._save_helper.writeFloatMat(np.array([self._class_list]), 'Class_List.DAP')
    self._save_helper.writeFloatMat(np.array([self._class_grip_data]), 'Grip_Data.DAP')

    os.chdir(work_dir)

  def getPropSpeed(self):
    tmp = np.mean(self._feats[0:8])
    tmp_2 = int(tmp/5)
    if tmp_2 > 255:
      tmp_2 = 255
    return tmp_2

  def classifierAvailable(self):
    return self._classifier_available

  def extractMAVFeats(self, data):
    tmp_1 = abs(data)
    tmp_1 = np.mean(tmp_1,axis=0)
    self._feats[0,0:8] = tmp_1

  def makeMAVBytes(self):
    counter = 1
    for x in range(8):
      int_feats = divmod(int(self._feats[0,x]),256)
      self._mav_bytes[counter] = int_feats[1]
      counter = counter + 1
      self._mav_bytes[counter] = int_feats[0]
      counter = counter + 1

  def extractWLFeats(self, data):
    tmp_2 = np.diff(data,axis=0)
    tmp_2 = abs(tmp_2)
    tmp_2 = np.mean(tmp_2,axis=0)
    self._feats[0,8:16] = tmp2

  def slowExtractWLFeats(self, data):
    dum = data.shape()
    tmp_2 = np.zeros((dum))
    for x in range(dum[0] - 1):
      tmp_2[x, :] = data[x, :] - data[x + 1, :]
    tmp_2 = abs(tmp_2)
    tmp_2 = np.mean(tmp_2, axis=0)
    self._feats[0, 8:16] = tmp_2

  def extractZCFeats(self,data):
    (m,n) = data.shape()
    tmp_u = (data>0)
    tmp_d = (data<0)
    res_zc = np.zeros((1,8))
    for x in range(n):
      t_u = sum(tmp_u[:][x])
      t_d = sum(tmp_d[:][x])
      if t_u > t_d:
        res_zc[0,x] = t_d
      else:
        res_zc[0,x] = t_u
    self._feats[0,16:24] = res_zc

  def extractSSCFeats(self, data):
    tmp = np.diff(data,axis=0)
    (m,n) = tmp.shape()
    tmp_u = (tmp>0)
    tmp_d = (tmp<0)
    res_zc = np.zeros((1,8))
    for x in range(n):
      t_u = sum(tmp_u[:][x])
      t_d = sum(tmp_d[:][x])
      if t_u > t_d:
        res_zc[0,x] = t_d
      else:
        res_zc[0,x] = t_u
    self._feats[0,24:32] = res_zc

  def slowExtractSSCFeats(self, data):
    dum = data.shape()
    tmp = np.zeros((dum))
    for x in range(dum[0] - 1):
      tmp[x, :] = data[x, :] - data[x + 1, :]

    (m, n) = tmp.shape()
    tmp_u = (tmp > 0)
    tmp_d = (tmp < 0)
    res_zc = np.zeros((1, 8))
    for x in range(n):
      t_u = sum(tmp_u[:][x])
      t_d = sum(tmp_d[:][x])
      if t_u > t_d:
        res_zc[0, x] = t_d
      else:
        res_zc[0, x] = t_u
    self._feats[0, 24:32] = res_zc

  def setCalibration(self, mov_num):
    if mov_num != -1:
      self._calibrating_flag = True
      self._calibration_class = mov_num
    else:
      self._calibrating_flag = False
      self._calibration_class = mov_num

  def checkCalibrationStatus(self):
    return self._calibration_class

  def extractTDFeats(self, data):
    data = data + np.zeros(data.shape(),dtype=np.int16)
    data = data - 32768
    self.extractMAVFeats(data)
    self.slowExtractWLFeats(data)
    self.extractZCFeats(data)
    self.slowExtractSSCFeats(data)
    self._feat_hist_buffer[self._feat_hist_count,:] = self._feats
    if not self._is_training:
      tmp = np.mean(self._feat_hist_buffer,axis=0)
    else:
      if not self._use_raw_data:
        #print("FILTERED DATA")
        tmp = np.mean(self._feat_hist_buffer,axis=0)
      else:
        #print("RAW DATA")
        pass
    self._feat_hist_count = self._feat_hist_count + 1
    if self._feat_hist_count >= 10:
      self._feat_hist_count = 0
    self._feats[0,0:32] = tmp
    self.makeMAVBytes()

  def getMAVBytes(self):
    return self._mav_bytes