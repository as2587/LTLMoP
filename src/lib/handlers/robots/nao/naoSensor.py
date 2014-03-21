#!/usr/bin/env python
"""
====================================================
naoSensors.py - Sensor handler for the Aldebaran Nao
====================================================
"""
import logging

class naoSensorHandler:
    def __init__(self, proj, shared_data):
        self.naoInitHandler = shared_data['NAO_INIT_HANDLER']

        self.sttProxy = None
        self.sttVocabulary = []
        self.sttVocabCounter = 0

        self.faceProxy = None
        self.memProxy = None
        self.sttProxy = None
        self.ldmProxy = None

        self.landMarkInitialized = False
        self.detectedVals = {}

        #for example:
        self.detectedVals[0] = [68, 130] #detectedVals[0] corresponds to values detected by Detector_0, Landmarks

    ###################################
    ### Available sensor functions: ###
    ###################################
    def _initLandMark(self):
        if not self.landMarkInitialized:
        # initialize landmark detection
            if self.ldmProxy == None:
                self.ldmProxy = self.naoInitHandler.createProxy('ALLandMarkDetection')
            if self.memProxy is None:
                self.memProxy = self.naoInitHandler.createProxy('ALMemory')

        ### Initialize land Mark tracking
            subs = [x[0] for x in self.ldmProxy.getSubscribersInfo()]

        # Close any previous subscriptions that might have been hanging open
            if "ltlmop_sensorhandler" in subs:
                self.ldmProxy.unsubscribe("ltlmop_sensorhandler")
            self.ldmProxy.subscribe("ltlmop_sensorhandler", 100, 0.0)
            self.landMarkInitialized=True

    def _getLandMarkNum(self):
        foundMarks=[]
        val = self.memProxy.getData("LandmarkDetected",0)
        if(val and isinstance(val, list) and len(val)>=2):
            markInfoArray = val[1]
            try:
                #msg = "Number of Markers found: " + str(len(markInfoArray))
                #msg = "length of foundMarks: "
                for markInfo in markInfoArray:
                    markExtraInfo = markInfo[1] #Number of each tag found
                    foundMarks.append(markExtraInfo[0])
                    #msg += "-- " + str(len(foundMarks))

                #logging.info(msg)
                return foundMarks
            except Exception, e:
                print "Naomarks detected, but it seems getData is invalid. ALValue ="
                print val
                print "Error msg %s" % (str(e))


    def seeLandMark(self,landMark_id,initial=False):
        """
        Use Nao's landmark recognition system to detect radial bar code landmark.
        For info about avaible bar code, refer to http://www.aldebaran-robotics.com/documentation/naoqi/vision/allandmarkdetection.html#allandmarkdetection

        landMark_id (int): The id number of bar code to detect
        """
        if initial:
            self._initLandMark()
            return True
        else:
            allFound = self._getLandMarkNum()
            if (isinstance(allFound, list) and len(allFound)>0):
                if (landMark_id in allFound):
                    return True
            return False

    def DetectLandmark(self, detectNum, initial=False):
        """
        Detector Function for Open-world specifications

        detectNum (int): A unique number to define the detector
        methodname (string): name of method to call
        """
        if initial:
            self._initLandMark()
        else:
            allFound = self._getLandMarkNum()
            if(isinstance(allFound, list) and len(allFound)>0):
                logging.info("DetectLandmark: " + str(len(allFound)))
                for elem in allFound:
                    if not (elem in self.detectedVals[detectNum]):
                        logging.info("Found an un-identified tag: " + str(elem))
                        #Add_to list...
                        #return True
                    else:
                        logging.info("Got one for " + str(elem))
            #         return True
        logging.info("----------")
        return False




    def hearWord(self, word, threshold, initial=False):
        """
        Use Nao's speech recognition system to detect a spoken word.

        word (string): The word to detect
        threshold (float): Minimum acceptable detection confidence (default=0.2,min=0,max=1)
        """

        if initial:
            ### Initialize speech-to-text

            if self.memProxy is None:
                self.memProxy = self.naoInitHandler.createProxy('ALMemory')

            if self.sttProxy is None:
                self.sttProxy = self.naoInitHandler.createProxy('ALSpeechRecognition')

            # Close any previous subscriptions that might have been hanging open
            subs = [x[0] for x in self.sttProxy.getSubscribersInfo()]
            if "ltlmop_sensorhandler" in subs:
                self.sttProxy.unsubscribe("ltlmop_sensorhandler")

            self.sttVocabulary += [word]
            self.sttProxy.setWordListAsVocabulary(self.sttVocabulary)

            self.sttProxy.setLanguage("English")
            self.sttProxy.setAudioExpression(False)
            self.sttProxy.setVisualExpression(True)
            self.sttProxy.subscribe("ltlmop_sensorhandler")

            # Reset the speech recognition register manually
            self.memProxy.insertData("WordRecognized", [])

            return True
        else:
            # Check speech recognition state

            wds = self.memProxy.getData("WordRecognized",0)

            # HACK: reset the speech recognition register manually once per vocab-cycle
            self.sttVocabCounter += 1
            if self.sttVocabCounter == len(self.sttVocabulary):
                self.memProxy.insertData("WordRecognized", [])
                self.sttVocabCounter = 0

            for wd, prob in zip(wds[0::2], wds[1::2]):
                if wd == word and prob > threshold:
                    print "Recognized word '%s' with p = %f" % (wd, prob)
                    return True

            return False


    def seePerson(self, initial=False):
        """
        Use Nao's face recognition to detect a person's face in the field of view.
        """

        if initial:
            ### Initialize face tracking

            if self.faceProxy is None:
                self.faceProxy = self.naoInitHandler.createProxy('ALFaceDetection')

                subs = [x[0] for x in self.faceProxy.getSubscribersInfo()]
                # Close any previous subscriptions that might have been hanging open
                if "ltlmop_sensorhandler" in subs:
                    self.faceProxy.unsubscribe("ltlmop_sensorhandler")
                self.faceProxy.subscribe("ltlmop_sensorhandler")

                return True
        else:
            # Check face detection state
            face_data = self.memProxy.getData("FaceDetected",0)
            return (face_data != [])

    def headTapped(self, initial=False):
        """
        Check whether the button on top of Nao's head is pressed.
        """

        if initial:
            if self.memProxy is None:
                self.memProxy = self.naoInitHandler.createProxy('ALMemory')
            return True
        else:
            return bool(self.memProxy.getData('FrontTactilTouched',0))

