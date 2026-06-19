# Cordeau OPTW Benchmark Results

## Experiment 1: Metaheuristics vs. MCTS vs. Pulse Comparison

**Settings**: 1 vehicle · 60 s timeout · single run (seed 42)

`*` Pulse values proven optimal by B&B completing before timeout.

| Instance | Greedy | RandGreedy | ILS09 | ILS-RR | LNS | GRASP+VNS | MCTS | Pulse | Opt |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| pr01 | 69 | 304 | 301 | 304 | 292 | 304 | 239 | 308* | 308 |
| pr02 | 74 | 374 | 376 | 376 | 385 | 402 | 295 | 404* | 404 |
| pr03 | 242 | 371 | 377 | 390 | 390 | 393 | 293 | 394* | 394 |
| pr04 | 194 | 429 | 454 | 436 | 432 | 469 | 329 | 489* | 489 |
| pr05 | 220 | 487 | 525 | 498 | 551 | 560 | 347 | 595* | 595 |
| pr06 | 244 | 455 | 508 | 508 | 522 | 578 | 402 | 591* | 591 |
| pr07 | 185 | 276 | 283 | 292 | 274 | 293 | 263 | 298* | 298 |
| pr08 | 180 | 430 | 445 | 453 | 423 | 458 | 379 | 463* | 463 |
| pr09 | 101 | 390 | 425 | 402 | 438 | 468 | 324 | 493* | 493 |
| pr10 | 123 | 487 | 508 | 508 | 526 | 576 | 478 | 594* | 594 |
| pr11 | 128 | 323 | 330 | 330 | 330 | 343 | 256 | 353* | 353 |
| pr12 | 106 | 391 | 407 | 422 | 426 | **434** | 293 | 430 | — |
| pr13 | 150 | 398 | 397 | 429 | 435 | 454 | 257 | 467* | 467 |
| pr14 | 124 | 458 | 495 | 495 | 478 | **537** | 350 | 500 | — |
| pr15 | 168 | 526 | 552 | 552 | 518 | **636** | 433 | 550 | — |
| pr16 | 220 | 495 | 575 | 508 | **590** | **610** | 387 | 583 | — |
| pr17 | 125 | 322 | 346 | 346 | 345 | 359 | 306 | 362* | 362 |
| pr18 | 263 | 406 | 417 | 417 | 424 | 451 | 332 | 539* | 539 |
| pr19 | 255 | 407 | 415 | 415 | **457** | **487** | 386 | 389 | — |
| pr20 | 202 | 543 | 585 | 543 | 566 | **618** | 410 | 599 | — |
| **TOTAL** | **3373** | **8272** | **8721** | **8624** | **8802** | **9430** | **6759** | **9401** | — |

Bold values beat Pulse. GRASP+VNS does so on 6 instances (pr12, pr14, pr15, pr16, pr19, pr20) — all cases where Pulse hit the 90 s timeout without proving optimality. LNS also beats Pulse on pr16 and pr19.


## Experiment 2: MCTS Enhancements
**Settings**: Cordeau OPTW (1 vehicle · 60 s · seed=42 · 1000 iter)


- **E2** = eliminate duplicate repair 
- **E5** = post-rollout LS  
- **E4** = informed expansion (alpha=3, rcl_size=3) is the best after testing `alpha` and `rcl_size` over {1, 2, 3, 4} × {2, 3, 5, 8, 10} (20 combinations).


| Instance | Baseline | +E2 | E2+E5 | E2+E5+E4 | dE2 | dE5 | dFinal | Pulse | Gap |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| pr01 | 239 | 235 | 265 | 279 | -4 | +26 | +40 | 308 | 9.4% |
| pr02 | 295 | 300 | 342 | 367 | +5 | +47 | +72 | 404 | 9.2% |
| pr03 | 293 | 307 | 353 | 357 | +14 | +60 | +64 | 394 | 9.4% |
| pr04 | 329 | 362 | 399 | 417 | +33 | +70 | +88 | 489 | 14.7% |
| pr05 | 347 | 388 | 429 | 469 | +41 | +82 | +122 | 595 | 21.2% |
| pr06 | 402 | 318 | 410 | 471 | -84 | +8 | +69 | 591 | 20.3% |
| pr07 | 263 | 247 | 249 | 264 | -16 | -14 | +1 | 298 | 11.4% |
| pr08 | 379 | 387 | 389 | 410 | +8 | +10 | +31 | 463 | 11.4% |
| pr09 | 324 | 348 | 373 | 384 | +24 | +49 | +60 | 493 | 22.1% |
| pr10 | 478 | 401 | 407 | 483 | -77 | -71 | +5 | 594 | 18.7% |
| pr11 | 256 | 252 | 270 | 313 | -4 | +14 | +57 | 353 | 11.3% |
| pr12 | 293 | 316 | 331 | 342 | +23 | +38 | +49 | 430 | 20.5% |
| pr13 | 257 | 295 | 360 | 392 | +38 | +103 | +135 | 467 | 16.1% |
| pr14 | 350 | 389 | 417 | 427 | +39 | +67 | +77 | 500 | 14.6% |
| pr15 | 433 | 411 | 448 | 504 | -22 | +15 | +71 | 550 | 8.4% |
| pr16 | 387 | 403 | 460 | 460 | +16 | +73 | +73 | 583 | 21.1% |
| pr17 | 306 | 272 | 286 | 314 | -34 | -20 | +8 | 362 | 13.3% |
| pr18 | 332 | 347 | 359 | 393 | +15 | +27 | +61 | 539 | 27.1% |
| pr19 | 386 | 398 | 400 | 392 | +12 | +14 | +6 | 389 | -0.8% |
| pr20 | 410 | 477 | 508 | 500 | +67 | +98 | +90 | 599 | 16.5% |
| **TOTAL** | **6759** | **6853** | **7455** | **7938** | **+94** | **+696** | **+1179** | | |


- **E1: Fix backpropagation (max → running average)** - no effect

Theoretically correct: UCB1's exploitation term should estimate expected reward, not the optimistic maximum. 
But no effect after testing at both 1k and 10k iterations: all 20 instances tied exactly under both the `max` and running-average formulations. The tree stays very shallow — with O(n) branching factor and ≤10k iterations, most nodes accumulate very few visits. At low visit counts the UCB1 exploration term `sqrt(2·log(N)/n)` dominates and the difference between `max` and `mean` never influences selection decisions. E1 would matter only if nodes were visited hundreds of times, which would require orders of magnitude more iterations than the 60 s budget allows.

- **E6: extract_best() after the main loop** - no effect

`extract_best()` greedily follows the child with the highest average value at each tree level and completes the solution with `repair()`. After E5, every rollout is already polished by `minimize_makespan` + `replace`, so individual simulations are near-optimal for the branches they explore. The tree's aggregate path never produced a solution better than what the rollouts had already stored. E5 made each rollout expensive and high-quality, rendering the aggregate signal redundant — E6 is effectively obsolete once E5 is in place.