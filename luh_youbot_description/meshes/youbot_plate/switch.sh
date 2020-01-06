# switch between origin and kai's version

mkdir temp

mv plate.dae temp
mv plate_convex.dae temp
mv plate_backup.dae temp
mv plate_convex_backup.dae temp

mv temp/plate.dae ./plate_backup.dae
mv temp/plate_convex.dae ./plate_convex_backup.dae
mv temp/plate_backup.dae ./plate.dae
mv temp/plate_convex_backup.dae ./plate_convex.dae

rm -r temp

echo "switched dae and backup dae!"