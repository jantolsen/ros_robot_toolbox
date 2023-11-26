// Template Class
// -------------------------------
// Description:
//      Template class file 
//      Empty class to be used as a template for creating new class-files
//
// Version:
//  0.1 - Initial Version
//        [26.11.2023]  -   Jan T. Olsen
//
// -------------------------------

// Include Header-files:
// -------------------------------
    
    // Class Header-File
    #include "robot_toolbox/template/template_class.h"

// Namespace: Template
// -------------------------------
namespace Template
{
    // Template Class
    // -------------------------------

        // Constants
        // -------------------------------
        const std::string Template::CLASS_PREFIX = "TemplateClass::";


        // Class constructor
        // -------------------------------
        // (Constructor Overloading)
        TemplateClass::TemplateClass()
        {
            // Initialize Template-Class
            init();
        } // Class Constructor End: TemplateClass()


        // Class Desctructor
        // -------------------------------
        TemplateClass::~TemplateClass()
        {
            // Report to terminal
            ROS_INFO_STREAM(CLASS_PREFIX << __FUNCTION__ 
                << ": Destructor called");

        } // Class Desctructor End: ~TemplateClass()


        // Initialize Template-Class
        // -------------------------------
        void TemplateClass::init()
        {
            // Report to terminal
            ROS_INFO_STREAM(CLASS_PREFIX << __FUNCTION__ 
                << ": Initializing class");
        } // Function End: init()


} // End Namespace: Template