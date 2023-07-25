
(cl:in-package :asdf)

(defsystem "iri_object_transportation_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "explicit_information" :depends-on ("_package_explicit_information"))
    (:file "_package_explicit_information" :depends-on ("_package"))
    (:file "localForcesCoefficients" :depends-on ("_package_localForcesCoefficients"))
    (:file "_package_localForcesCoefficients" :depends-on ("_package"))
    (:file "localForcesSFM" :depends-on ("_package_localForcesSFM"))
    (:file "_package_localForcesSFM" :depends-on ("_package"))
    (:file "narrowPathMarkersArray" :depends-on ("_package_narrowPathMarkersArray"))
    (:file "_package_narrowPathMarkersArray" :depends-on ("_package"))
    (:file "twistStamped" :depends-on ("_package_twistStamped"))
    (:file "_package_twistStamped" :depends-on ("_package"))
    (:file "twistStampedArray" :depends-on ("_package_twistStampedArray"))
    (:file "_package_twistStampedArray" :depends-on ("_package"))
    (:file "wrenchStampedArray" :depends-on ("_package_wrenchStampedArray"))
    (:file "_package_wrenchStampedArray" :depends-on ("_package"))
    (:file "wrenchStampedWithCoeff" :depends-on ("_package_wrenchStampedWithCoeff"))
    (:file "_package_wrenchStampedWithCoeff" :depends-on ("_package"))
    (:file "wrenchWithCoeffArray" :depends-on ("_package_wrenchWithCoeffArray"))
    (:file "_package_wrenchWithCoeffArray" :depends-on ("_package"))
  ))