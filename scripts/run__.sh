# gnome-terminal --window -e 'bash -c "source ./run1.sh; sleep 1 ; exec bash"' --tab -e 'bash -c "source ./run2.sh ; sleep 3 ; exec bash"'
# gnome-terminal --title="new title" --tab -e 'bash -c "source ./run2.sh ; sleep 3 ; exec bash"'

# gnome-terminal --title="new title" --tab -e 'bash run_gazebo.sh'