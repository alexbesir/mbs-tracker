 bbs = MBS_track(imgFileNames,bb,options)
==========================================

  A moving-background subtraction based object tracker.
 
  Inputs:
 
     imgFileNames ... a cell array with N elements of image filenames
                      that represent individual frames of the input video
 
     bb ............. bounding box of the target object's location in the
                      first frame in format [x,y,w,h] where x, y are the
                      coordinates of the upper-left corner and w, h are the
                      width and height
 
     options ........ a struct for defining options (see below for
                      available options and their default values). To keep
                      all options values default, options can be an empty
                      struct
 
  Outputs:
 
     bbs ............ the resulting matrix of bounding boxes that represent
                      the tracked target object's positions. The matrix
                      dimensions are [Nx4], where each row represents a
                      single frame. Each bounding box is specified in the
                      form of [x,y,w,h] where x, y are the coordinates of
                      the upper-left corner and w, h are the width and
                      height
 
  Otpions:
 
     skipTracking ... [false] or true. If true, the last step (tracking)
                      will be skipped. The function will return a NaN.
 
     saveTD ......... [false] or <filename>. If defined, the original video
                      with the tracked object's location in each frame will
                      be saved to an mp4 file specified by the value of
                      this option (ex. 'tracked.avi')
 
     saveFG ......... [false] or <filename>. If defined, the extracted
                      foreground will be saved to an mp4 file specified by
                      the value of this option (ex. 'fore.avi')
 
     saveBG ......... [false] or <filename>. If defined, the subtracted
                      background will be saved to an mp4 file specified by
                      the value of this option (ex. 'back.avi')
 
  Written by Alexander Besir (alex.besir@gmail.com)
  Faculty of Computer and Information Science Ljubljana
  May, 2014