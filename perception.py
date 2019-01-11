import numpy as np
import cv2

# Identify pixels above the threshold
# Threshold of RGB > 160 does a nice job of identifying ground pixels only
def color_thresh(img, rgb_thresh=(170, 170, 165)):
    # Create an array of zeros same xy size as img, but single channel
    color_select = np.zeros_like(img[:,:,0])
    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met
    above_thresh = (img[:,:,0] > rgb_thresh[0]) \
                & (img[:,:,1] > rgb_thresh[1]) \
                & (img[:,:,2] > rgb_thresh[2])
    # Index the array of zeros with the boolean array and set to 1
    color_select[above_thresh] = 1
    # Return the binary image
    return color_select

def below_thresh(img, rgb_thresh=(65, 50, 50)):
    # Create an array of zeros same xy size as img, but single channel
    obstacle_select = np.zeros_like(img[:,:,0])
    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met
    below_thresh = (img[:,:,0] < rgb_thresh[0]) \
                & (img[:,:,1] < rgb_thresh[1]) \
                & (img[:,:,2] < rgb_thresh[2])
    # Index the array of zeros with the boolean array and set to 1
    obstacle_select[below_thresh] = 1
    # Return the binary image
    return obstacle_select

def gold_thresh(img):
    # Create an array of zeros same xy size as img, but single channel
    gold_select = np.zeros_like(img[:,:,0])
    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met
    gold_thresh = (img[:,:,0] > 110) \
                & (img[:,:,0] < 255) \
                & (img[:,:,1] > 110) \
                & (img[:,:,1] < 255) \
                & (img[:,:,2] >= 0) \
                & (img[:,:,2] < 60)
    # Index the array of zeros with the boolean array and set to 1
    gold_select[gold_thresh] = 1
    # Return the binary image
    return gold_select

# Define a function to convert from image coords to rover coords
def rover_coords(binary_img):
    # Identify nonzero pixels
    ypos, xpos = binary_img.nonzero()
    # Calculate pixel positions with reference to the rover position being at the 
    # center bottom of the image.  
    x_pixel = -(ypos - binary_img.shape[0]).astype(np.float)
    y_pixel = -(xpos - binary_img.shape[1]/2 ).astype(np.float)
    return x_pixel, y_pixel


# Define a function to convert to radial coords in rover space
def to_polar_coords(x_pixel, y_pixel):
    # Convert (x_pixel, y_pixel) to (distance, angle) 
    # in polar coordinates in rover space
    # Calculate distance to each pixel
    dist = np.sqrt(x_pixel**2 + y_pixel**2)
    # Calculate angle away from vertical for each pixel
    angles = np.arctan2(y_pixel, x_pixel)
    #obs_dists=[]
    #for x in range (len(angles)):
        #if angles[x] <5 and angles[x] > -5:
            #obs_dists.append(dist[x])
    return dist, angles#, obs_dists

# Define a function to map rover space pixels to world space
def rotate_pix(xpix, ypix, yaw):
    # Convert yaw to radians
    yaw_rad = yaw * np.pi / 180
    xpix_rotated = (xpix * np.cos(yaw_rad)) - (ypix * np.sin(yaw_rad))
                            
    ypix_rotated = (xpix * np.sin(yaw_rad)) + (ypix * np.cos(yaw_rad))
    # Return the result  
    return xpix_rotated, ypix_rotated

def translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale): 
    # Apply a scaling and a translation
    xpix_translated = (xpix_rot / scale) + xpos
    ypix_translated = (ypix_rot / scale) + ypos
    # Return the result  
    return xpix_translated, ypix_translated


# Define a function to apply rotation and translation (and clipping)
# Once you define the two functions above this function should work
def pix_to_world(xpix, ypix, xpos, ypos, yaw, world_size, scale):
    # Apply rotation
    xpix_rot, ypix_rot = rotate_pix(xpix, ypix, yaw)
    # Apply translation
    xpix_tran, ypix_tran = translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale)
    # Perform rotation, translation and clipping all at once
    x_pix_world = np.clip(np.int_(xpix_tran), 0, world_size - 1)
    y_pix_world = np.clip(np.int_(ypix_tran), 0, world_size - 1)
    # Return the result
    return x_pix_world, y_pix_world

# Define a function to perform a perspective transform
def perspect_transform(img, src, dst):
           
    M = cv2.getPerspectiveTransform(src, dst)
    warped = cv2.warpPerspective(img, M, (img.shape[1], img.shape[0]))# keep same size as input image
    
    return warped


# Apply the above functions in succession and update the Rover state accordingly
def perception_step(Rover):
    # Perform perception steps to update Rover()
    # TODO: 
    # NOTE: camera image is coming to you in Rover.img
    # 1) Define source and destination points for perspective transform
    img = Rover.img
    dst_size = 5 
    bottom_offset = 6
    source = np.float32([[14, 140], [301 ,140],[200, 96], [118, 96]])
    destination = np.float32([[img.shape[1]/2 - dst_size, img.shape[0] - bottom_offset],
                  [img.shape[1]/2 + dst_size, img.shape[0] - bottom_offset],
                  [img.shape[1]/2 + dst_size, img.shape[0] - 2*dst_size - bottom_offset], 
                  [img.shape[1]/2 - dst_size, img.shape[0] - 2*dst_size - bottom_offset],
                  ])
    # 2) Apply perspective transform
    warped = perspect_transform(img,source,destination)
    # 3) Apply color threshold to identify navigable terrain/obstacles/rock samples
    nav_terrain = color_thresh(warped)
    obs_terrain = below_thresh(warped)
    gold = gold_thresh(warped)
    
    vision_nav = color_thresh(img)
    vision_obs = below_thresh(img)
    vision_gld = gold_thresh(img)
    # 4) Update Rover.vision_image (this will be displayed on left side of screen)
    Rover.vision_image[:,:,0] = vision_obs*255 
    Rover.vision_image[:,:,1] = vision_gld*255
    Rover.vision_image[:,:,2] = vision_nav*255 

    # 5) Convert map image pixel values to rover-centric coords
    nav_x,nav_y = rover_coords(nav_terrain)
    obs_x,obs_y = rover_coords(obs_terrain)
    gld_x,gld_y = rover_coords(gold)
    # 6) Convert rover-centric pixel values to world coordinates
    nav_x_world,nav_y_world = pix_to_world(nav_x,nav_y,Rover.pos[0],Rover.pos[1],Rover.yaw,200,10)
    obs_x_world,obs_y_world = pix_to_world(obs_x,obs_y,Rover.pos[0],Rover.pos[1],Rover.yaw,200,10)
    gld_x_world,gld_y_world = pix_to_world(gld_x,gld_y,Rover.pos[0],Rover.pos[1],Rover.yaw,200,10)
    # 7) Update Rover worldmap (to be displayed on right side of screen)
        # Example: Rover.worldmap[obstacle_y_world, obstacle_x_world, 0] += 1
        #          Rover.worldmap[rock_y_world, rock_x_world, 1] += 1
        #          Rover.worldmap[navigable_y_world, navigable_x_world, 2] += 1
    if (Rover.roll < 0.1 or Rover.roll > 359.9)and (Rover.pitch <0.1 or Rover.pitch > 359.9):
        Rover.worldmap[obs_y_world,obs_x_world,0] = 100
        Rover.worldmap[nav_y_world,nav_x_world,2] = 255
    #Rover.nav_terr[nav_y_world,nav_x_world,2]+= 1
    #Rover.nav_terr[obs_y_world,obs_x_world,2]-= 1
        nav_terr = Rover.worldmap[:,:,2] > 0
        Rover.worldmap[nav_terr,0]=0
    #trth_terr = Rover.ground_truth[:,:,1] == 0
    #Rover.worldmap[trth_terr,2]=0
    
    #If any values for gold rocks have been identified:
    if gold.any():
        #First, the gold rock coordinates are converted to 
        #angles and distances
        gld_dists,gld_angles = to_polar_coords(gld_x,gld_y)
        #Next, the index location of the minimum distance in the list
        #(i.e. the part of the gold rock closest to the camera and,
        # therefore, the most accurate predictor of the rock's location)
        # is made into a variable.
        gld_idx = np.argmin(gld_dists)
        #Lastly, the position of the gold rock is redefined to only the 
        #minimum distance point, and the world map is updated with these
        #coordinates in all three channels of the worldmap.
        gld_x_pos = gld_x_world[gld_idx]
        gld_y_pos = gld_y_world[gld_idx]
        Rover.worldmap[gld_y_pos,gld_x_pos,:]=255
    # 8) Convert rover-centric pixel positions to polar coordinates
    nav_dists,nav_angles = to_polar_coords(nav_x,nav_y)
    obs_dists,obs_angles = to_polar_coords(obs_x,obs_y)
    
    #First, a variable is defined, same length as 'nav_angles', where values
    #are true only where 'nav_angles' is equal to zero.
    x_angles = nav_angles == 0
    #Next, a list of distances is compiled from 'nav_dists' corresponding to
    #only where the angles are zero (i.e. the list of distances directly
    #in front of the rover).
    zero_dists = nav_dists[x_angles]
    #An identical method is performed as above, only this time only for 
    #distances at greater than -15 degrees (important for later, since
    #since the rover will be intentionally trending towards the right).
    far_angles = nav_angles < -0.26
    far_dists = nav_dists[far_angles]
    # Update Rover pixel distances and angles
        # Rover.nav_dists = rover_centric_pixel_distances
        # Rover.nav_angles = rover_centric_angles
    if gold.any():
        Rover.gld_dists = gld_dists
        Rover.gld_angles = gld_angles
        #Rover.nav_obs_dists = gld_obs_dists
        #Rover.for_gold = True
    else:
        Rover.gld_dists = None
        Rover.gld_angles = None
    
    if nav_terrain.any():
        Rover.nav_dists = nav_dists
        Rover.nav_angles = nav_angles
    else:
        Rover.nav_dists = [1]
        Rover.nav_angles = 1
        
    if zero_dists.any():
        Rover.zero_dists = zero_dists
    else:
        Rover.zero_dists = [1]
    if far_dists.any():
        Rover.far_dists = far_dists
    else:
        Rover.far_dists = [1]
    #Rover.nav_obs_dists = nav_obs_dists
    
    
    
    return Rover