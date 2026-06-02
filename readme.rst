MVUtensils
==========

MVUtensils is a large refactoring and cleanup of the original VapourSynth MVTools port
with the goals of fixing several long standing bugs in the original such as the right and bottom
border not being properly processed. As a result of this many opertions should also be considerably faster since more CPU cache friendly
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