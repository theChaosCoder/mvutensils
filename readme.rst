MVUtensils
==========

MVUtensils is a large refactoring and cleanup of the original VapourSynth MVTools port
with the goals of fixing several long standing bugs in the original such as the right and bottom
border not being properly processed. As a result of this many operations should also be considerably faster since more CPU cache friendly
algorithms are being used.

Usage hints
===========

Super/Recalculate:
	* Recalculate only needs a single level super clip so remember to pass levels=1 to save memory and speed things up

Changes
=======

* All:
    * The namespace is now mvu
    
    * A prefix argument has been added to change the prefix of attached properties
    
    * blksize has been renamed to blksizeh and overlap to overlaph, they still implicitly set blksizev and overlapv like before
    
    * Frame properties are now consistently propagated, this means that many functions that previously only requested frames from super now also request from the original clip

	* The opt argument was dropped from all functions

* Super:
    * Now takes blksize=[h, v], overlap=[h, v] values to properly pad the source frame so the edges also are processed and not generate excessive levels that are unused

    * blksize and overlap are mandatory arguments

    * The pad arguments also got changed to pad=[h, v], if no value is provided it uses the default, if only one value is provided it's used for both horizontal and vertical

	* Greatly reduced memory usage

* Analyse/Recalculate:
    * Now takes blksize=[h, v] and overlap=[h, v] like Super. These values also default to the ones used when creating the super clip.

    * The dct argument was replaced with satd (true/false), satd=false is equivalent to dct=0 and satd=true is the same as dct=5 in the original mvtools

    * The rfilter argument was changed and mode 1/3 was removed, as a result the new mapping is 2=>1 and 4=>2
    
    * The search and pelsearch argument had mode 0 and 1 removed, as a result all remaining mode have been adjusted by -2
	
	* Removed the divide argument since no good use case seems to exist, in recalculate Recalculate you can already get an effect similar to divide=1/2 depending on smooth=False/True
    
    * The isb argument was removed, instead delta accepts both positive and negative numbers to indicate direction
    
    * Will throw an error when not all pixels can be processed due to the chosen blksize/overlap combination in contrast to the values used in Super. Note that generally halving blksize+overlap and reusing the Super clip will work. Other more esoteric splits may or may not require a new Super clip to be derived.
    
	* Renamed lambda to mvlamda and global to globalmv to not collide with python keywords
	
* AnalyseMany:
	* A helper function to generate a multiple analysis clips quickly to pass to DegrainN and friends, takes the same arguments as Analyse except that delta is a positive number controlling the step size backward and forward. The radius argument determines how many vectors are produced. For example radius=2 will return [Analyse(delta=1), Analyse(delta=-1), Analyse(delta=2), Analyse(delta=-2)]
	
* Compensate:
    * None
    
* DegrainN:
    * All the cryptically named forward and backward vector clip arguments is now passed as an array in vectors

	* Greatly reduced memory usage

* Degrain:
	* A convenience function that deduces the correct DegrainN call from the number of passed vectors, created to combine with AnalyseMany

* Mask:
	* Split into 3 functions that correspond to kind 0-2 called VectorLengthMask, SADMask and OcclusionMask, obviously the kind argument has been removed
	
	* The clip argument was removed, it was used for nothing at all except attaching source frame properties and as such is pointless
	
	* Returns a grayscale full range mask only instead of the weird UV plane stuff goign on in the original
	
	* Supports 8-16 bit, the output format is derived entirely from the vector clip
	
	* The masks are actually generated at a higher bitdepth unlike avs+ where the same 8 bit mask is always upscaled

* Flow:	
	* Improved internal mask resizing quality
	
	* Greatly reduced memory usage
	
	* Mode=1 was dropped since nobody used it, since only mode=0 remains the mode argument was removed

* FlowBlur	
	* Improved internal mask resizing quality
	
	* Greatly reduced memory usage
	
* FlowFPS
	* Improved internal mask resizing quality
	
	* Greatly reduced memory usage

	* The mask argument was replaced with extramask=True/False, extramask=False is the same as mask=1 and extramask=True is equivalent to mask=2

* SCDetection:
    * None
	
* BlockFPS:
	* Removed since nobody uses it and FlowFPS is generally both the better and more popular option

* Finest:
	* Removed since its only real use was as a support function for other filters due to lazy frame data access code

Planned changes/mysteries
=========================

* Does the delta sign direction make sense? Flip it?

* Have FlowBlur, FlowFPS and FlowInter take a vectors=[bw, fw] argument instead

* Big additional code cleanups

* Depan