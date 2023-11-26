// Template Class
// -------------------------------
// Description:
//      Template class header file 
//      Empty class to be used as a template for creating new header-files
//
// Version:
//  0.1 - Initial Version
//        [26.11.2023]  -   Jan T. Olsen
//
// -------------------------------

// Include guard:
// -------------------------------
// Prevents double declaration of identifiers (e.g. types, enums, static variables)
//  #ifndef: 
//      Check whether header-file with the unique value "xxx_H" is already included
//  #define: 
//      If header-file not earlier included, it continues and defines the rest of the file 
//  #endif: 
//      End of include guard
#ifndef TEMPLATE_CLASS_H       
#define TEMPLATE_CLASS_H

// Include Header-files:
// -------------------------------
    // Standard
    #include <iostream>
    #include <string>
    #include <vector>

    // Ros
    #include <ros/ros.h>

    // Robotics Toolbox
    #include <robot_toolbox/toolbox.h>

// Namespace: Template
// -------------------------------
namespace Template
{
    // Constants
    // -------------------------------
        const int TEMPLATE_CONST = 0;

    // Structs
    // -------------------------------
        // Template-Struct    
        struct TemplateStruct
        {
            const int id;               // identifer
            const std::string name;     // name
        };

    // Enums
    // -------------------------------
        // Template-Enum
        enum TemplateEnum
        {
            X = 0,
            Y = 1,
            Z = 2
        };

    // Type definitions
    // -------------------------------

    // Template Class
    // -------------------------------
    /** \brief Description on the Template-Class
    */
    class TemplateClass
    {
        // Public Class members
        // -------------------------------
        // Accessible for everyone
        public:

            // Class constructor
            // -------------------------------
            /** \brief Template-Class Constuctor
            * \param param Class param [data-type]
            */
            TemplateClass();


            // Class destructor
            // -------------------------------
            /** \brief Template-Class destructor
            */
            ~TemplateClass();


        // Protected Class members
        // -------------------------------
        // Accessible within the class which defines them, 
        // and classes which inherits from the parent class
        protected:

            // Initialize Template-Class
            // -------------------------------
            /** \brief Initialize Template-Class
            */
            void init();


        // Private Class members
        // -------------------------------
        // Accessible only for the class which defines them
        private:
            // Class-Name-Prefix for terminal message
            static const std::string CLASS_PREFIX;


    }; // End Class: TemplateClass
} // End Namespace: Template
#endif // TEMPLATE_CLASS_H 