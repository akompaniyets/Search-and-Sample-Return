import numpy as np
import time
import random


def decision_step(Rover):
    if Rover.gld_angles is not None and Rover.mode == 'forward':
        if np.mean(Rover.gld_dists)<13:
            Rover.throttle = 0
            #Rover.mode = 'gold'
            Rover.brake = Rover.brake_set
            if Rover.near_sample and Rover.vel == 0 and not Rover.picking_up:
                #Rover.mode = 'forward'
                Rover.send_pickup = True
        else:
            if Rover.vel <= 0.6:
                Rover.throttle = Rover.throttle_set
                Rover.brake = 0
                Rover.steer = np.clip((np.mean(Rover.gld_angles))* 180/np.pi, -15, 15)
            else:
                Rover.throttle = 0
                Rover.brake = 0
                Rover.steer = np.clip((np.mean(Rover.gld_angles))* 180/np.pi, -15, 15)
    else:    
        if Rover.nav_angles is not None and Rover.mode == 'forward':# and Rover.for_gold == False:
            #if np.mean(Rover.nav_angles)>0:
                #if Rover.vel <= Rover.max_vel:
                    #Rover.throttle = Rover.throttle_set
                    #Rover.brake = 0
                    #Rover.steer = -np.clip((np.mean(Rover.nav_angles))* 180/np.pi, -15, 15)
                #else:
                    #Rover.throttle = 0
                    #Rover.brake = 0
                    #Rover.steer = -np.clip((np.mean(Rover.nav_angles))* 180/np.pi, -15, 15)
            if np.mean(Rover.nav_dists)<20 or len(Rover.nav_angles)<200 or np.max(Rover.zero_dists) < 20 or np.max(Rover.far_dists) < 10:# or np.min(Rover.nav_dists)>20:
                Rover.throttle = 0
                #Rover.brake = Rover.brake_set
                Rover.mode = 'stop'
        #elif Rover.vel == 0 and(np.absolute(np.mean(Rover.nav_angles)* 180/np.pi) > 10):
            #Rover.brake = 0
            #Rover.throttle = 0
            #Rover.steer = 15
            else:
                if Rover.vel <= Rover.max_vel:
                    Rover.throttle = Rover.throttle_set
                    Rover.brake = 0
                    if np.mean(Rover.nav_angles)>0:
                        Rover.steer = 0.5*np.clip((np.mean(Rover.nav_angles))* 180/np.pi, -15, 15)-6.5
                    else:
                        Rover.steer = np.clip((np.mean(Rover.nav_angles))* 180/np.pi, -15, 15)-5
                else:
                    Rover.throttle = 0
                    Rover.brake = 0
                    if np.mean(Rover.nav_angles)>0:
                        Rover.steer = 0.5*np.clip((np.mean(Rover.nav_angles))* 180/np.pi, -15, 15)-6.5
                    else:
                        Rover.steer = np.clip((np.mean(Rover.nav_angles))* 180/np.pi, -15, 15)-5
        elif Rover.nav_angles is not None and Rover.mode == 'stop':
            if Rover.vel > 0.2:
                Rover.brake = Rover.brake_set
                Rover.throttle = 0
            else:
                Rover.brake = 0
                Rover.throttle = 0
                if np.mean(Rover.nav_dists)<30 or len(Rover.nav_angles)<800 or np.max(Rover.zero_dists) < 25 or np.max(Rover.far_dists) < 20:
                    Rover.steer = 15
                else:
                    Rover.mode = 'forward'
        elif np.absolute(Rover.steer) == 15 and Rover.throttle == 0.2 and Rover.vel <=0.2 and Rover.mode == 'forward':
            Rover.mode = 'obstacle'
        elif Rover.nav_angles is not None and Rover.mode == 'obstacle':
            Rover.throttle = 0
            Rover.brake = 0
            Rover.steer = 15
            if Rover.yaw == random.uniform(0,(2*np.pi)):
                Rover.mode='forward'
        else:
            if Rover.vel > 0.2:
                Rover.brake = Rover.brake_set
                Rover.throttle = 0
            else:
                Rover.brake = 0
                Rover.throttle = 0
                Rover.mode = 'stop'
                #Rover.steer = 15
    return Rover

# Unused code:
    #elif Rover.gld_angles is not None and Rover.mode == 'gold':
        #if Rover.near_sample and Rover.vel == 0 and not Rover.picking_up:
            #Rover.mode = 'forward'
            #Rover.send_pickup = True
            
        #elif Rover.near_sample and Rover.vel >= 0 and not Rover.picking_up:
            #Rover.throttle = 0
            #Rover.brake = Rover.brake_set
    #elif Rover.mode == 'gold' and Rover.gld_angles == None:
        #Rover.brake = 0
        #Rover.throttle = 0
        #if np.mean(Rover.nav_dists) < 30:
            #Rover.steer = 15
        #else:
            #Rover.mode = 'forward'

        #elif Rover.gld_angles is not None and Rover.mode == 'forward':# and Rover.for_gold == True:
                #Rover.brake = Rover.brake_set
        #Rover.mode = 'forward'
    #else:
        #Rover.throttle = Rover.throttle_set
        #Rover.brake = 0
        #Rover.steer = np.clip((np.mean(Rover.nav_angles))* 180/np.pi, -15, 15)
    
    # Example:
    # Check if we have vision data to make decisions with
    #if Rover.for_gold == True and Rover.mode == 'forward':
        #if not Rover.near_sample:
            #Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
            #while Rover.vel < Rover.max_vel:
                #Rover.throttle = Rover.throttle_set
            #Rover.brake = 0
        #else:
            #Rover.throttle = 0
                    # Set brake to stored brake value
            #Rover.brake = Rover.brake_set
            #Rover.steer = 0
            #Rover.mode = 'stop'
            #Rover.send_pickup = True
    #if Rover.nav_angles is not None: #and Rover.for_gold == False:
        # Check for Rover.mode status
        #if Rover.mode == 'forward': 
            # Check the extent of navigable terrain
            #if len(Rover.nav_angles) >= Rover.stop_forward:  
            #if np.max(Rover.nav_dists) >= 50:
                # If mode is forward, navigable terrain looks good 
                # and velocity is below max, then throttle 
                #if Rover.vel < Rover.max_vel:
                    # Set throttle value to throttle setting
                    #Rover.throttle = Rover.throttle_set
                #else: # Else coast
                    #Rover.throttle = 0
                #Rover.brake = 0
                # Set steering to average angle clipped to the range +/- 15
                #Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
            # If there's a lack of navigable terrain pixels then go to 'stop' mode
            #elif len(Rover.nav_angles) < Rover.stop_forward:
            #elif np.max(Rover.nav_dists) < 50:
                    # Set mode to "stop" and hit the brakes!
                    #Rover.throttle = 0
                    # Set brake to stored brake value
                    #Rover.brake = Rover.brake_set
                    #Rover.steer = 0
                    #Rover.mode = 'stop'

        # If we're already in "stop" mode then make different decisions
        #elif Rover.mode == 'stop':
            # If we're in stop mode but still moving keep braking
            #if Rover.vel > 0.2:
                #Rover.throttle = 0
                #Rover.brake = Rover.brake_set
                #Rover.steer = 0
            # If we're not moving (vel < 0.2) then do something else
            #elif Rover.vel <= 0.2:
                # Now we're stopped and we have vision data to see if there's a path forward
                #if len(Rover.nav_angles) < Rover.go_forward:
                #if np.max(Rover.nav_obs_dists) < 2:
                    #Rover.throttle = 0
                    # Release the brake to allow turning
                    #Rover.brake = 0
                    # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
                    #Rover.steer = -15 # Could be more clever here about which way to turn
                # If we're stopped but see sufficient navigable terrain in front then go!
                #if len(Rover.nav_angles) >= Rover.go_forward:
                #if np.max(Rover.nav_obs_dists) > 2:
                    # Set throttle back to stored value
                    #Rover.throttle = Rover.throttle_set
                    # Release the brake
                    #Rover.brake = 0
                    # Set steer to mean angle
                    #Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
                    #Rover.mode = 'forward'
    # Just to make the rover do something 
    # even if no modifications have been made to the code
    #else:
        #Rover.throttle = Rover.throttle_set
        #Rover.steer = 0
        #Rover.brake = 0
        
    # If in a state where want to pickup a rock send pickup command
    #if Rover.near_sample and Rover.vel == 0 and not Rover.picking_up:
        #Rover.send_pickup = True
    #elif Rover.near_sample and Rover.vel >= 0 and not Rover.picking_up:
        #Rover.throttle = 0
        #Rover.brake = Rover.brake_set
        