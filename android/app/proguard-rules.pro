# kotlinx.serialization
-keepattributes *Annotation*, InnerClasses
-dontnote kotlinx.serialization.AnnotationsKt
-keepclassmembers class kotlinx.serialization.json.** { *** Companion; }
-keepclasseswithmembers class kotlinx.serialization.json.** { kotlinx.serialization.KSerializer serializer(...); }
-keep,includedescriptorclasses class com.uzls.four.**$$serializer { *; }
-keepclassmembers class com.uzls.four.** { *** Companion; }
-keepclasseswithmembers class com.uzls.four.** { kotlinx.serialization.KSerializer serializer(...); }
