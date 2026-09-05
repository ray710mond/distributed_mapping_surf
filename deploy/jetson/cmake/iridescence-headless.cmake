# Make Iridescence's imported target available to GLIM and its consumers while
# keeping GLIM's incompatible optional GUI modules disabled. GLIM conditionally
# links this target whenever it already exists.
find_package(Iridescence QUIET)
