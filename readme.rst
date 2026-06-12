MVUtensils
==========

MVUtensils is a large refactoring and cleanup of the original VapourSynth MVTools port
with the goals of fixing several long standing bugs in the original such as the right and bottom
border not being properly processed. As a result of this many operations should also be considerably faster since more CPU cache friendly
algorithms are being used.

Changes
=======

* All:
    * The namespace is now mvu
    
    * A prefix argument has been added to change the prefix of attached properties
    
    * blksize has been renamed to blksizeh and overlap to overlaph, they still implicitly set blksizev and overlapv like before
    
    * Frame properties are now consistently propagated, this means that many functions that previously only requested frames from super now also request from the original clip
    

* Super:
    * Now takes blksizeh/v and overlaph/v values to properly pad the source frame so the edges also are processed and not generate excessive levels that are unused, these are basically mandatory and may be changed to be later

	* Reduced memory usage

* Analyse/Recalculate:
    * The dct argument was replaced with satd (true/false), satd=false is equivalent to dct=0 and satd=true is the same as dct=5 in the original mvtools

    * The rfilter argument was changed and mode 1/3 was removed, as a result the new mapping is 2=>1 and 4=>2
    
    * The search and pelsearch argument had mode 0 and 1 removed, as a result all remaining mode have been adjusted by -2
    
    * The isb argument was removed, instead delta accepts both positive and negative numbers to indicate direction
    
    * Will throw an error when not all pixels can be processed due to the chosen blocksize/overlap combination
    
* Compensate:
    * None
    
* DegrainN:
    * All the cryptically named forward and backward vector clip arguments is now passed as an array in vectors

* Mask:
	* Split into 3 functions that correspond to kind 0-2 called VectorLengthMask, SADMask and OcclusionMask, obviously the kind argument has been removed
	
	* The clip argument was removed, it was used for nothing at all except attaching source frame properties and as such is pointless
	
	* Returns a grayscale full range mask only instead of the weird UV plane stuff goign on in the original
	
	* Supports 8-16 bit, the output format is derived entirely from the vector clip
	
	* The masks are actually generated at a higher bitdepth unlike avs+ where the same 8 bit mask is always upscaled

* Flow:
	* 8-16 bit support
	
	* Improved internal mask resizing quality
	
	* Reduced memory usage
	
	* Mode=1 was dropped since nobody used it, since only mode=0 remains and of course the mode argument was removed

* SCDetection:
    * None

Planned changes
===============

* Remove the divide argument in Analyse/Recalculate if nobody can explain why it's useful, Recalculate is already similar to divide=1/2 depending on smooth=False/True

* Have an Analyse wrapper that outputs a full array of all clips required for DegrainN

* Make block size and overlap compulsory in Super (probably)

* Make overlaph/v and blksizeh/v simply blocksize=[w, h] and overlap=[w, h] arrays

* Make overlap and blksize default to the values in the super clip for analyse (maybe)

* Have a general Degrain function that maps to the right DegrainN depending on the number of vectors passed (maybe, feedback welcome)

* Remove Finest (only exists as a helper function due to poor super frame data layout)

* Slowly port all other bits