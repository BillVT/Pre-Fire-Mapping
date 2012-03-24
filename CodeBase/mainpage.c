/*! \mainpage Pre-Fire-Mapping System
 *  
 * @authors William Etter (MSE '11)
 *
 * <hr>
 *
 * @section intro Introduction
 * This system utilizes a Hokuyo LIDAR Laser Rangefinder mounted on 
 * a handheld unit to scan a location and save the data to a flashdrive.
 * This data is then uploaded to the base station where the SLAM algorithm 
 * (run in ROS - Robot Operating System).  From here the scanned area (rooms,
 * floors, buildings) can be visualized and input to other programs such as 
 * 3D modelings and building databases.
 *
 *
 * @section components Components
 * This system utilzies the following subsystems and components:
 * 
 * - Remote System (Handheld Scanning Unit):
 * -# <A HREF="http://www.hokuyo-aut.jp/02sensor/07scanner/urg_04lx.html"> Hokuyo URG-04LX-UG01(Simple-URG) </A>
 * -# <A HREF="http://beagleboard.org/bone"> BeagleBone </A> (Low-cost, fan-less single board computer) with custom Linux install
 * -# <A HREF="http://www.chrobotics.com/index.php?main_page=product_info&products_id=2CHRobotics"> CHR-6dm Inertial Measurement Unit </A>
 * -# <A HREF="http://www.4dsystems.com.au/prod.php?id=114"> 4D uLCD-32PTGFX </A> 3.2inch TFT Touchscreen LCD Display
 * -# <A HREF="http://www.newegg.com/Product/Product.aspx?Item=N82E16817394106"> Belkin 4-Port USB Hub </A>
 * -# <A HREF="www.sparkfun.com/products/9718"> 2x FTDI Cables </A>
 * -# <A HREF="http://www.castlecreations.com/products/ccbec.html"> Castle Creations 5V 10A BEC </A> (Voltage Regulator)
 * -# Portable Flash Drive
 * -# Lithium-Polymer Battery
 *
 * - Base System

 *
 *
 * <hr>
 *
 *
 * @section website Project Website: 
 * <A HREF="http://williametter.com/portfolio/projects/pre-fire-mapping-system/"> Pre-Fire-Mapping System </A>
 *
 *
 *
 * <hr>
 * @section notes Release.Notes
 * - Release Note #1
 *
 * <hr>
 * 
 * @todo Complete LIDAR API
 * @todo Determine LIDAR data save format for .bag file in ROS
 * @todo Implement LCD Screen
 * @todo Implement IMU System
 *
 * <hr>
 *
 * \section install_sec Installation
 *
 * \subsection step1 Step 1: Installing Custom Linux onto BeagleBone
 * -# Download files from Project Repository
 */
