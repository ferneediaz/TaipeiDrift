# Result tables

## m5_clouds

| route | point | filter | RMSE med [m] | CEP50 [m] | CEP95 [m] | final err med [m] | converged | diverged | false fix | NEES med | ground echo % | ms/step |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| A | grade=navigation__cloud_fraction=0.0 | MPF baseline (last echo) | 3880 | 247 | 9881 | 10457 | 90% | 80% | 95% | 2.7 | 33 | 3.05 |
| A | grade=navigation__cloud_fraction=0.0 | MPF gated | 10 | 8 | 20 | 19 | 90% | 10% | 10% | 0.4 | 33 | 2.79 |
| A | grade=navigation__cloud_fraction=0.0 | MPF proposed (obscuration-aware) | 10 | 7 | 18 | 11 | 100% | 0% | 5% | 0.6 | 33 | 3.35 |
| A | grade=navigation__cloud_fraction=0.0 | TERCOM | 77 | 22 | 154 | 21 | 100% | 0% | 80% | 1.3 | 33 | 0.05 |
| A | grade=navigation__cloud_fraction=0.1 | MPF baseline (last echo) | 22279 | 2752 | 49834 | 57931 | 80% | 85% | 95% | 61.8 | 30 | 3.99 |
| A | grade=navigation__cloud_fraction=0.1 | MPF gated | 20 | 8 | 45 | 28 | 90% | 30% | 45% | 0.5 | 30 | 4.10 |
| A | grade=navigation__cloud_fraction=0.1 | MPF proposed (obscuration-aware) | 57 | 8 | 71 | 29 | 100% | 25% | 45% | 0.6 | 30 | 4.42 |
| A | grade=navigation__cloud_fraction=0.1 | TERCOM | 819 | 45 | 1738 | 1197 | 75% | 50% | 90% | 4.6 | 30 | 0.08 |
| A | grade=navigation__cloud_fraction=0.3 | MPF baseline (last echo) | 39355 | 19242 | 86829 | 104554 | 40% | 100% | 100% | 599613.5 | 25 | 3.91 |
| A | grade=navigation__cloud_fraction=0.3 | MPF gated | 9399 | 1581 | 21936 | 21218 | 85% | 60% | 75% | 3713.2 | 25 | 3.03 |
| A | grade=navigation__cloud_fraction=0.3 | MPF proposed (obscuration-aware) | 1780 | 19 | 4248 | 5813 | 90% | 60% | 75% | 1.7 | 25 | 3.33 |
| A | grade=navigation__cloud_fraction=0.3 | TERCOM | 27693 | 19773 | 46884 | 47545 | 25% | 100% | 100% | 12608.3 | 25 | 0.07 |
| A | grade=navigation__cloud_fraction=0.5 | MPF baseline (last echo) | 84257 | 43631 | 185950 | 209254 | 25% | 100% | 100% | 26791158.6 | 20 | 2.41 |
| A | grade=navigation__cloud_fraction=0.5 | MPF gated | 17066 | 4259 | 37596 | 45214 | 80% | 90% | 95% | 11051.9 | 20 | 2.07 |
| A | grade=navigation__cloud_fraction=0.5 | MPF proposed (obscuration-aware) | 12998 | 3159 | 32545 | 41224 | 75% | 90% | 100% | 236.7 | 20 | 2.15 |
| A | grade=navigation__cloud_fraction=0.5 | TERCOM | 27842 | 25205 | 45940 | 47505 | 35% | 100% | 100% | 5926.8 | 20 | 0.04 |
| A | grade=navigation__cloud_fraction=0.7 | MPF baseline (last echo) | 74198 | 35560 | 152960 | 169943 | 5% | 100% | 100% | 2412065.1 | 14 | 1.85 |
| A | grade=navigation__cloud_fraction=0.7 | MPF gated | 23406 | 9543 | 48918 | 58830 | 50% | 100% | 100% | 20075.6 | 14 | 1.67 |
| A | grade=navigation__cloud_fraction=0.7 | MPF proposed (obscuration-aware) | 29472 | 7601 | 65847 | 77081 | 40% | 100% | 100% | 1916.8 | 14 | 1.88 |
| A | grade=navigation__cloud_fraction=0.7 | TERCOM | 41692 | 32604 | 72933 | 77917 | 5% | 100% | 100% | 15594.1 | 14 | 0.03 |
| A | grade=navigation__cloud_fraction=0.9 | MPF baseline (last echo) | 73436 | 36528 | 154643 | 170044 | 0% | 100% | 100% | 1730851.7 | 9 | 1.83 |
| A | grade=navigation__cloud_fraction=0.9 | MPF gated | 31402 | 11486 | 68434 | 75163 | 25% | 100% | 100% | 45411.0 | 9 | 1.61 |
| A | grade=navigation__cloud_fraction=0.9 | MPF proposed (obscuration-aware) | 33003 | 12260 | 74213 | 87498 | 30% | 100% | 100% | 5365.4 | 9 | 1.74 |
| A | grade=navigation__cloud_fraction=0.9 | TERCOM | 51542 | 46167 | 82254 | 87608 | 0% | 100% | 100% | 40177.5 | 9 | 0.03 |
| A | grade=tactical__cloud_fraction=0.0 | MPF baseline (last echo) | 2240 | 148 | 5964 | 6672 | 95% | 60% | 85% | 2.7 | 33 | 3.37 |
| A | grade=tactical__cloud_fraction=0.0 | MPF gated | 16 | 13 | 28 | 33 | 85% | 15% | 30% | 1.3 | 33 | 3.03 |
| A | grade=tactical__cloud_fraction=0.0 | MPF proposed (obscuration-aware) | 15 | 12 | 28 | 24 | 85% | 15% | 35% | 1.6 | 33 | 3.46 |
| A | grade=tactical__cloud_fraction=0.0 | TERCOM | 15431 | 1344 | 35926 | 42484 | 90% | 80% | 100% | 163.5 | 33 | 0.06 |
| A | grade=tactical__cloud_fraction=0.1 | MPF baseline (last echo) | 14853 | 3961 | 28670 | 38578 | 80% | 90% | 100% | 941.9 | 30 | 2.67 |
| A | grade=tactical__cloud_fraction=0.1 | MPF gated | 27 | 16 | 44 | 38 | 80% | 30% | 45% | 2.2 | 30 | 2.18 |
| A | grade=tactical__cloud_fraction=0.1 | MPF proposed (obscuration-aware) | 95 | 17 | 224 | 40 | 90% | 30% | 60% | 2.1 | 30 | 2.46 |
| A | grade=tactical__cloud_fraction=0.1 | TERCOM | 36674 | 15583 | 73407 | 84294 | 60% | 95% | 100% | 7596.1 | 30 | 0.04 |
| A | grade=tactical__cloud_fraction=0.3 | MPF baseline (last echo) | 46578 | 28882 | 84896 | 93744 | 35% | 100% | 100% | 368304.3 | 25 | 2.38 |
| A | grade=tactical__cloud_fraction=0.3 | MPF gated | 33465 | 6665 | 77978 | 96445 | 70% | 90% | 100% | 5913.6 | 25 | 2.38 |
| A | grade=tactical__cloud_fraction=0.3 | MPF proposed (obscuration-aware) | 5273 | 882 | 12741 | 17243 | 90% | 70% | 95% | 12.8 | 25 | 2.64 |
| A | grade=tactical__cloud_fraction=0.3 | TERCOM | 46609 | 29297 | 86366 | 94353 | 25% | 100% | 100% | 37290.0 | 25 | 0.03 |
| A | grade=tactical__cloud_fraction=0.5 | MPF baseline (last echo) | 46646 | 27905 | 82756 | 92279 | 5% | 100% | 100% | 1625596.5 | 19 | 2.95 |
| A | grade=tactical__cloud_fraction=0.5 | MPF gated | 35873 | 18856 | 78688 | 91150 | 60% | 90% | 100% | 11088.5 | 19 | 2.71 |
| A | grade=tactical__cloud_fraction=0.5 | MPF proposed (obscuration-aware) | 37624 | 16511 | 84715 | 97612 | 60% | 95% | 100% | 3304.6 | 19 | 3.23 |
| A | grade=tactical__cloud_fraction=0.5 | TERCOM | 47443 | 28971 | 89269 | 95122 | 5% | 100% | 100% | 12772.6 | 19 | 0.05 |
| A | grade=tactical__cloud_fraction=0.7 | MPF baseline (last echo) | 54891 | 39705 | 89790 | 107824 | 0% | 100% | 100% | 2196973.0 | 14 | 4.46 |
| A | grade=tactical__cloud_fraction=0.7 | MPF gated | 58720 | 22095 | 119708 | 129662 | 40% | 95% | 100% | 12435.5 | 14 | 3.54 |
| A | grade=tactical__cloud_fraction=0.7 | MPF proposed (obscuration-aware) | 23603 | 13474 | 48822 | 54080 | 55% | 100% | 100% | 2345.0 | 14 | 3.90 |
| A | grade=tactical__cloud_fraction=0.7 | TERCOM | 43205 | 34379 | 70522 | 74355 | 15% | 100% | 100% | 34843.7 | 14 | 0.09 |
| A | grade=tactical__cloud_fraction=0.9 | MPF baseline (last echo) | 57260 | 38383 | 100147 | 114773 | 0% | 100% | 100% | 974606.3 | 9 | 4.48 |
| A | grade=tactical__cloud_fraction=0.9 | MPF gated | 60641 | 29647 | 121167 | 126886 | 25% | 100% | 100% | 10405.8 | 9 | 3.50 |
| A | grade=tactical__cloud_fraction=0.9 | MPF proposed (obscuration-aware) | 39954 | 22759 | 74865 | 82943 | 10% | 100% | 100% | 6610.4 | 9 | 3.96 |
| A | grade=tactical__cloud_fraction=0.9 | TERCOM | 50995 | 35732 | 104262 | 112566 | 0% | 100% | 100% | 19023.3 | 9 | 0.10 |
| C | grade=navigation__cloud_fraction=0.0 | MPF baseline (last echo) | 3534 | 398 | 9505 | 11676 | 100% | 75% | 85% | 2.4 | 41 | 2.59 |
| C | grade=navigation__cloud_fraction=0.0 | MPF gated | 93 | 11 | 85 | 26 | 100% | 5% | 5% | 0.3 | 41 | 2.47 |
| C | grade=navigation__cloud_fraction=0.0 | MPF proposed (obscuration-aware) | 110 | 9 | 74 | 31 | 100% | 0% | 5% | 0.5 | 41 | 2.84 |
| C | grade=navigation__cloud_fraction=0.0 | TERCOM | 74609 | 63091 | 125004 | 131639 | 0% | 100% | 100% | 7328.1 | 41 | 0.07 |
| C | grade=navigation__cloud_fraction=0.1 | MPF baseline (last echo) | 29035 | 14047 | 59514 | 69239 | 45% | 95% | 95% | 11344.9 | 35 | 2.72 |
| C | grade=navigation__cloud_fraction=0.1 | MPF gated | 113 | 13 | 146 | 41 | 90% | 20% | 25% | 0.4 | 35 | 2.50 |
| C | grade=navigation__cloud_fraction=0.1 | MPF proposed (obscuration-aware) | 187 | 12 | 280 | 45 | 100% | 20% | 35% | 0.6 | 35 | 3.21 |
| C | grade=navigation__cloud_fraction=0.1 | TERCOM | 82809 | 62920 | 139745 | 147091 | 0% | 100% | 100% | 21876.4 | 35 | 0.07 |
| C | grade=navigation__cloud_fraction=0.3 | MPF baseline (last echo) | 75796 | 34202 | 158477 | 174457 | 5% | 100% | 100% | 369006.1 | 25 | 2.84 |
| C | grade=navigation__cloud_fraction=0.3 | MPF gated | 11684 | 1420 | 28744 | 32927 | 75% | 75% | 85% | 62.6 | 25 | 2.50 |
| C | grade=navigation__cloud_fraction=0.3 | MPF proposed (obscuration-aware) | 5598 | 294 | 14373 | 17114 | 75% | 70% | 95% | 3.8 | 25 | 2.79 |
| C | grade=navigation__cloud_fraction=0.3 | TERCOM | 62543 | 54955 | 103602 | 108558 | 0% | 100% | 100% | 16826.0 | 25 | 0.08 |
| C | grade=navigation__cloud_fraction=0.5 | MPF baseline (last echo) | 67986 | 28670 | 130389 | 141635 | 10% | 100% | 100% | 582953.1 | 20 | 2.81 |
| C | grade=navigation__cloud_fraction=0.5 | MPF gated | 19310 | 7228 | 37416 | 50893 | 45% | 95% | 95% | 5211.8 | 20 | 2.68 |
| C | grade=navigation__cloud_fraction=0.5 | MPF proposed (obscuration-aware) | 16286 | 7163 | 38587 | 43845 | 60% | 90% | 100% | 438.2 | 20 | 3.03 |
| C | grade=navigation__cloud_fraction=0.5 | TERCOM | 61808 | 57174 | 105957 | 112766 | 0% | 100% | 100% | 14541.5 | 20 | 0.08 |
| C | grade=navigation__cloud_fraction=0.7 | MPF baseline (last echo) | 67931 | 38087 | 128139 | 138231 | 0% | 100% | 100% | 404291.6 | 12 | 2.69 |
| C | grade=navigation__cloud_fraction=0.7 | MPF gated | 22789 | 9909 | 44541 | 48661 | 45% | 100% | 100% | 6009.8 | 12 | 2.53 |
| C | grade=navigation__cloud_fraction=0.7 | MPF proposed (obscuration-aware) | 18668 | 9453 | 36354 | 42684 | 35% | 100% | 100% | 834.1 | 12 | 2.92 |
| C | grade=navigation__cloud_fraction=0.7 | TERCOM | 64779 | 50309 | 99603 | 104759 | 0% | 100% | 100% | 1582.5 | 12 | 0.11 |
| C | grade=navigation__cloud_fraction=0.9 | MPF baseline (last echo) | 91328 | 44046 | 199344 | 217214 | 0% | 100% | 100% | 459785.2 | 3 | 2.55 |
| C | grade=navigation__cloud_fraction=0.9 | MPF gated | 21547 | 10461 | 39528 | 48057 | 20% | 100% | 100% | 8855.3 | 3 | 2.33 |
| C | grade=navigation__cloud_fraction=0.9 | MPF proposed (obscuration-aware) | 32449 | 18010 | 58340 | 64205 | 5% | 100% | 100% | 4347.8 | 3 | 2.51 |
| C | grade=navigation__cloud_fraction=0.9 | TERCOM | 57419 | 58408 | 86348 | 91738 | 0% | 100% | 100% | 1754.4 | 3 | 0.10 |
| C | grade=tactical__cloud_fraction=0.0 | MPF baseline (last echo) | 426 | 141 | 925 | 324 | 75% | 40% | 65% | 1.0 | 41 | 1.70 |
| C | grade=tactical__cloud_fraction=0.0 | MPF gated | 121 | 18 | 140 | 64 | 90% | 25% | 30% | 0.6 | 41 | 1.59 |
| C | grade=tactical__cloud_fraction=0.0 | MPF proposed (obscuration-aware) | 131 | 14 | 119 | 36 | 100% | 0% | 25% | 0.9 | 41 | 1.84 |
| C | grade=tactical__cloud_fraction=0.0 | TERCOM | 59864 | 54943 | 100491 | 116475 | 0% | 100% | 100% | 6305.0 | 41 | 0.05 |
| C | grade=tactical__cloud_fraction=0.1 | MPF baseline (last echo) | 45106 | 28310 | 81634 | 81968 | 25% | 100% | 100% | 23320.7 | 36 | 1.63 |
| C | grade=tactical__cloud_fraction=0.1 | MPF gated | 178 | 19 | 312 | 65 | 70% | 30% | 60% | 1.1 | 36 | 1.58 |
| C | grade=tactical__cloud_fraction=0.1 | MPF proposed (obscuration-aware) | 375 | 18 | 1080 | 46 | 75% | 30% | 65% | 1.4 | 36 | 1.76 |
| C | grade=tactical__cloud_fraction=0.1 | TERCOM | 78700 | 64079 | 134558 | 140333 | 0% | 100% | 100% | 5876.2 | 36 | 0.04 |
| C | grade=tactical__cloud_fraction=0.3 | MPF baseline (last echo) | 133158 | 60794 | 245348 | 260956 | 15% | 100% | 100% | 57228.7 | 29 | 1.67 |
| C | grade=tactical__cloud_fraction=0.3 | MPF gated | 13795 | 2469 | 31893 | 38109 | 80% | 75% | 95% | 551.9 | 29 | 1.66 |
| C | grade=tactical__cloud_fraction=0.3 | MPF proposed (obscuration-aware) | 24495 | 7853 | 50128 | 48334 | 60% | 65% | 95% | 248.9 | 29 | 1.83 |
| C | grade=tactical__cloud_fraction=0.3 | TERCOM | 68439 | 58702 | 122914 | 134554 | 0% | 100% | 100% | 7476.3 | 29 | 0.04 |
| C | grade=tactical__cloud_fraction=0.5 | MPF baseline (last echo) | 169677 | 97046 | 300393 | 311164 | 0% | 100% | 100% | 103281.2 | 18 | 1.70 |
| C | grade=tactical__cloud_fraction=0.5 | MPF gated | 42975 | 15216 | 80739 | 84103 | 35% | 100% | 100% | 1861.8 | 18 | 1.62 |
| C | grade=tactical__cloud_fraction=0.5 | MPF proposed (obscuration-aware) | 28685 | 17498 | 60019 | 68156 | 45% | 100% | 100% | 1079.5 | 18 | 1.77 |
| C | grade=tactical__cloud_fraction=0.5 | TERCOM | 94571 | 63002 | 164683 | 177111 | 0% | 100% | 100% | 5261.2 | 18 | 0.05 |
| C | grade=tactical__cloud_fraction=0.7 | MPF baseline (last echo) | 108433 | 73732 | 200070 | 211785 | 0% | 100% | 100% | 37556.0 | 9 | 1.99 |
| C | grade=tactical__cloud_fraction=0.7 | MPF gated | 33315 | 23418 | 60047 | 64433 | 10% | 100% | 100% | 2299.7 | 9 | 2.04 |
| C | grade=tactical__cloud_fraction=0.7 | MPF proposed (obscuration-aware) | 39820 | 25109 | 76841 | 82243 | 5% | 100% | 100% | 4992.8 | 9 | 2.29 |
| C | grade=tactical__cloud_fraction=0.7 | TERCOM | 93910 | 68759 | 146258 | 150250 | 0% | 100% | 100% | 10968.1 | 9 | 0.06 |
| C | grade=tactical__cloud_fraction=0.9 | MPF baseline (last echo) | 193398 | 141268 | 332163 | 338492 | 0% | 100% | 100% | 69782.9 | 2 | 2.68 |
| C | grade=tactical__cloud_fraction=0.9 | MPF gated | 51157 | 33990 | 94807 | 103889 | 5% | 100% | 100% | 5079.8 | 2 | 2.65 |
| C | grade=tactical__cloud_fraction=0.9 | MPF proposed (obscuration-aware) | 29759 | 25808 | 51636 | 54340 | 5% | 100% | 100% | 1663.5 | 2 | 2.96 |
| C | grade=tactical__cloud_fraction=0.9 | TERCOM | 72940 | 61654 | 151803 | 180240 | 0% | 100% | 100% | 6646.3 | 2 | 0.12 |

## m6_beams

| route | point | filter | RMSE med [m] | CEP50 [m] | CEP95 [m] | final err med [m] | converged | diverged | false fix | NEES med | ground echo % | ms/step |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| A | beam_set=nadir__cloud_fraction=0.0 | MPF baseline (last echo) | 3514 | 471 | 8432 | 8514 | 100% | 75% | 90% | 3.5 | 33 | 2.74 |
| A | beam_set=nadir__cloud_fraction=0.0 | MPF proposed (obscuration-aware) | 17 | 15 | 29 | 28 | 95% | 5% | 55% | 2.8 | 33 | 2.94 |
| A | beam_set=nadir__cloud_fraction=0.1 | MPF baseline (last echo) | 32743 | 7959 | 78563 | 91760 | 75% | 95% | 100% | 7026.1 | 31 | 2.57 |
| A | beam_set=nadir__cloud_fraction=0.1 | MPF proposed (obscuration-aware) | 27 | 16 | 54 | 40 | 95% | 15% | 55% | 2.5 | 31 | 2.71 |
| A | beam_set=slant3__cloud_fraction=0.0 | MPF baseline (last echo) | 45 | 13 | 33 | 37 | 100% | 10% | 65% | 2.7 | 33 | 4.05 |
| A | beam_set=slant3__cloud_fraction=0.0 | MPF proposed (obscuration-aware) | 18 | 11 | 29 | 16 | 100% | 0% | 75% | 2.8 | 33 | 4.97 |
| A | beam_set=slant3__cloud_fraction=0.1 | MPF baseline (last echo) | 26656 | 23506 | 48590 | 54268 | 55% | 95% | 100% | 18747079.5 | 30 | 3.38 |
| A | beam_set=slant3__cloud_fraction=0.1 | MPF proposed (obscuration-aware) | 39 | 13 | 46 | 30 | 100% | 15% | 90% | 4.0 | 30 | 3.39 |
| B | beam_set=nadir__cloud_fraction=0.0 | MPF baseline (last echo) | 37167 | 23588 | 68085 | 75451 | 0% | 100% | 100% | 703.2 | 73 | 1.62 |
| B | beam_set=nadir__cloud_fraction=0.0 | MPF proposed (obscuration-aware) | 37059 | 22088 | 58688 | 56395 | 0% | 100% | 100% | 45.9 | 73 | 1.91 |
| B | beam_set=nadir__cloud_fraction=0.1 | MPF baseline (last echo) | 47503 | 31007 | 81574 | 84517 | 0% | 100% | 100% | 3235.1 | 67 | 1.59 |
| B | beam_set=nadir__cloud_fraction=0.1 | MPF proposed (obscuration-aware) | 24606 | 18446 | 44264 | 49362 | 0% | 100% | 100% | 30.1 | 67 | 1.81 |
| B | beam_set=slant3__cloud_fraction=0.0 | MPF baseline (last echo) | 27090 | 21461 | 51151 | 61229 | 0% | 100% | 100% | 687.4 | 72 | 2.09 |
| B | beam_set=slant3__cloud_fraction=0.0 | MPF proposed (obscuration-aware) | 39381 | 31632 | 67093 | 67186 | 0% | 100% | 100% | 8851.7 | 72 | 2.47 |
| B | beam_set=slant3__cloud_fraction=0.1 | MPF baseline (last echo) | 58942 | 34240 | 107377 | 111562 | 0% | 100% | 100% | 9122.0 | 63 | 2.06 |
| B | beam_set=slant3__cloud_fraction=0.1 | MPF proposed (obscuration-aware) | 34623 | 21383 | 61406 | 65364 | 0% | 100% | 100% | 2385.7 | 63 | 2.43 |
| C | beam_set=nadir__cloud_fraction=0.0 | MPF baseline (last echo) | 720 | 200 | 1762 | 1542 | 95% | 50% | 85% | 1.2 | 41 | 1.59 |
| C | beam_set=nadir__cloud_fraction=0.0 | MPF proposed (obscuration-aware) | 124 | 13 | 114 | 30 | 100% | 0% | 15% | 0.9 | 41 | 1.82 |
| C | beam_set=nadir__cloud_fraction=0.1 | MPF baseline (last echo) | 62846 | 25580 | 142451 | 163174 | 30% | 95% | 100% | 19336.9 | 37 | 1.71 |
| C | beam_set=nadir__cloud_fraction=0.1 | MPF proposed (obscuration-aware) | 409 | 18 | 1038 | 79 | 90% | 40% | 65% | 1.1 | 37 | 1.93 |
| C | beam_set=slant3__cloud_fraction=0.0 | MPF baseline (last echo) | 200 | 18 | 446 | 103 | 85% | 15% | 70% | 1.5 | 41 | 2.44 |
| C | beam_set=slant3__cloud_fraction=0.0 | MPF proposed (obscuration-aware) | 45 | 10 | 53 | 26 | 95% | 5% | 45% | 1.5 | 41 | 2.99 |
| C | beam_set=slant3__cloud_fraction=0.1 | MPF baseline (last echo) | 36280 | 17919 | 65092 | 71676 | 60% | 80% | 100% | 39827.3 | 38 | 2.46 |
| C | beam_set=slant3__cloud_fraction=0.1 | MPF proposed (obscuration-aware) | 182 | 12 | 204 | 32 | 90% | 20% | 70% | 1.8 | 38 | 2.71 |

## m6_imu

| route | point | filter | RMSE med [m] | CEP50 [m] | CEP95 [m] | final err med [m] | converged | diverged | false fix | NEES med | ground echo % | ms/step |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| A | grade=mems | MPF baseline (last echo) | 228617 | 54872 | 458763 | 547745 | 40% | 100% | 100% | 184095.0 | 30 | 1.80 |
| A | grade=mems | MPF proposed (obscuration-aware) | 188716 | 36630 | 320306 | 335732 | 70% | 85% | 100% | 554.6 | 30 | 1.78 |
| A | grade=navigation | MPF baseline (last echo) | 37876 | 5635 | 81626 | 94504 | 60% | 95% | 100% | 5980.2 | 31 | 1.68 |
| A | grade=navigation | MPF proposed (obscuration-aware) | 16 | 8 | 23 | 18 | 100% | 25% | 30% | 0.6 | 31 | 1.83 |
| A | grade=tactical | MPF baseline (last echo) | 25340 | 11450 | 51007 | 52825 | 80% | 95% | 100% | 57899.6 | 31 | 1.79 |
| A | grade=tactical | MPF proposed (obscuration-aware) | 59 | 16 | 54 | 31 | 100% | 20% | 75% | 2.0 | 31 | 1.84 |
| B | grade=mems | MPF baseline (last echo) | 229495 | 111556 | 444414 | 462977 | 0% | 100% | 100% | 19789.7 | 67 | 1.72 |
| B | grade=mems | MPF proposed (obscuration-aware) | 63471 | 43716 | 121284 | 124052 | 0% | 100% | 100% | 85.7 | 67 | 2.01 |
| B | grade=navigation | MPF baseline (last echo) | 52562 | 18802 | 128714 | 152265 | 0% | 100% | 100% | 3941.1 | 65 | 1.65 |
| B | grade=navigation | MPF proposed (obscuration-aware) | 15990 | 7700 | 30998 | 32958 | 0% | 100% | 95% | 5.9 | 65 | 1.85 |
| B | grade=tactical | MPF baseline (last echo) | 54211 | 27647 | 112574 | 130686 | 0% | 100% | 100% | 3015.6 | 66 | 1.63 |
| B | grade=tactical | MPF proposed (obscuration-aware) | 33267 | 17576 | 67260 | 75670 | 0% | 100% | 100% | 39.7 | 66 | 1.92 |
| C | grade=mems | MPF baseline (last echo) | 201691 | 99222 | 392975 | 417807 | 15% | 100% | 100% | 71779.9 | 36 | 1.69 |
| C | grade=mems | MPF proposed (obscuration-aware) | 126456 | 42409 | 290190 | 307474 | 35% | 100% | 100% | 1258.2 | 36 | 1.63 |
| C | grade=navigation | MPF baseline (last echo) | 22190 | 8043 | 43015 | 45649 | 50% | 95% | 100% | 8486.1 | 36 | 1.69 |
| C | grade=navigation | MPF proposed (obscuration-aware) | 336 | 13 | 823 | 63 | 95% | 40% | 55% | 0.6 | 36 | 1.82 |
| C | grade=tactical | MPF baseline (last echo) | 61456 | 21475 | 131670 | 139163 | 45% | 100% | 100% | 14195.9 | 37 | 1.83 |
| C | grade=tactical | MPF proposed (obscuration-aware) | 169 | 16 | 326 | 49 | 95% | 25% | 40% | 1.0 | 37 | 1.99 |

## m6_baro

| route | point | filter | RMSE med [m] | CEP50 [m] | CEP95 [m] | final err med [m] | converged | diverged | false fix | NEES med | ground echo % | ms/step |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| A | enabled=False | MPF baseline (last echo) | 11092 | 1369 | 26174 | 29891 | 70% | 85% | 90% | 44.0 | 31 | 1.76 |
| A | enabled=False | MPF proposed (obscuration-aware) | 102 | 16 | 102 | 34 | 95% | 30% | 60% | 1.3 | 31 | 1.85 |
| A | enabled=True | MPF baseline (last echo) | 28577 | 11753 | 51695 | 62469 | 65% | 95% | 100% | 59330.9 | 30 | 1.75 |
| A | enabled=True | MPF proposed (obscuration-aware) | 123 | 16 | 149 | 56 | 100% | 25% | 55% | 1.8 | 30 | 1.82 |
| B | enabled=False | MPF baseline (last echo) | 40304 | 30821 | 77504 | 67169 | 0% | 100% | 100% | 7178.8 | 64 | 1.77 |
| B | enabled=False | MPF proposed (obscuration-aware) | 38259 | 18709 | 66209 | 67321 | 0% | 100% | 95% | 8.9 | 64 | 2.05 |
| B | enabled=True | MPF baseline (last echo) | 56871 | 45221 | 102875 | 116924 | 0% | 100% | 100% | 5260.8 | 66 | 2.02 |
| B | enabled=True | MPF proposed (obscuration-aware) | 44250 | 29558 | 70604 | 72814 | 0% | 100% | 100% | 91.1 | 66 | 2.31 |
| C | enabled=False | MPF baseline (last echo) | 30584 | 21255 | 58238 | 67681 | 55% | 100% | 100% | 11940.1 | 36 | 2.79 |
| C | enabled=False | MPF proposed (obscuration-aware) | 409 | 19 | 1053 | 429 | 90% | 45% | 60% | 1.6 | 36 | 2.95 |
| C | enabled=True | MPF baseline (last echo) | 39090 | 24865 | 75502 | 85372 | 35% | 90% | 95% | 20236.9 | 37 | 2.76 |
| C | enabled=True | MPF proposed (obscuration-aware) | 466 | 20 | 1086 | 79 | 75% | 45% | 75% | 1.6 | 37 | 3.18 |
