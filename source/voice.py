import pyautogui as pa
import time

class Voice():
    #generate list from string
    def genList(self, phrase):
        split_phrase = list(phrase)
        return split_phrase

    #click somewhere
    def click(self, x, y, button):
        pa.click(x, y, button=button)
        time.sleep(0.5)

    #type each letter in a given string
    def keyboardInput(self, phrase):
        for letter in phrase:
            pa.press(letter)

    def clearField(self, x, y):
        pa.click(x, y, button='left')
        pa.keyDown('ctrl')
        pa.press('a')
        pa.keyUp('ctrl')
        pa.press('backspace')

    def hitEnter(self):
        pa.press('enter')

    def Play(self, x, y):
        pa.click(x, y, button='left')

    def selVoice(self, x1, y1, x2, y2):
        pa.click(x1, y1, button='left')
        time.sleep(1)
        pa.click(x2, y2, button='left')

    def openChrome(self):
        self.click(32, 289, 'right')
        self.click(119, 291, 'left')

    def searchSite(self, site):
        self.click(2056, 71, 'left')
        self.keyboardInput(site)
        self.hitEnter()
        time.sleep(3)

    def Inp(self, text):
        self.keyboardInput(text)
        self.Play(2596, 749)
