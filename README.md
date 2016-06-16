Turku Ekotuki trainset controller
=================================

Pedal powered trainset controller, using Arduino and a bunch of other components.
Can also be run using wall-wart if required

This is very much work in progress at this time.

Ekotuki junarata suomeksi
=========================
Turun ekotuki on toteuttanut kuluvan vuoden 2015 aikana sähköä tuottavan kuntopyörän sekä siihen 
liitettävän pienoisrautatien. Rautatie on toiminnallinen kokonaisuus, johon liittyy erilaisia 
ympäristökasvatuksellisia teemoja.
Käsiteltävät teemat ovat energia, liikkuminen, jätehuolto ja luonnon monimuotoisuus. Jokaiseen 
teemaan kuuluu neljä teoriakorttia, sekä tehtäväkortteja. Teoriakortteja voi 
valita eri teemoista haluamiensa aihesisältöjen mukaan.

* Energia teemassa käsitellään uusiutuvia energiamuotoja, sekä energian fiksua käyttöä. Junaratamaisemasta löytyvät vesivoimaa kuvaava mylly, rakenteilla oleva tuulivoimala sekä ihan oikea aurinkopaneeli, jolla saa valaistua omakotitalon yläkerran. Lisäksi maisemassa on biokaasuvoimala.
* Liikkumisen teemaa tuodaan esille fölibussin, kävelyn ja pyöräilyn, junan ja sähköauton voimin.
* Jätehuolto-osiossa esitellään kerrostalon kierrätyspiste, jossa erikseen nostetaan vuoden 2016 alussa alkava biojätteen keräys. Biojäte kuljetetaan Ratakaupungissa biokaasuvoimalaan, jossa siitä valmistetaan energiaa.
* Luonnon monimuotoisuus käsittelee vieraslajeja, Varsinais-Suomen luontoa, ravintoketjuja ja ekosysteemipalveluita.



What does it do ?
=================
Takes 8-12V as input and runs a train set. The model also has lights for various purposes.

Requirements:
=============
* PWM controlled motor controller
* PWM controlled LED controller
* 2 Track sensors connected to IRQ pins
* INA219 for voltage and current measurements
* I2C connected character LCD for informational messages
* I2C connected OLED screen for sponsor information
* IR RX on pin 12

TODO
====
* Sounds
* Adjust lights in some way
* Add switch handling
