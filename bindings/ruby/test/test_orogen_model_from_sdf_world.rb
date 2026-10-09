# frozen_string_literal: true

require 'test/helpers'
require 'rock_gazebo/orogen_model_from_sdf_world'

module RockGazebo
    describe 'orogen_model_from_sdf_world' do
        before do
            require 'orocos'
            Orocos.load unless Orocos.loaded?

            @world = SDF::Root.load(
                File.join(data_dir, 'orogen_model_test.world'), flatten: false
            ).each_world.first
            @model = RockGazebo.orogen_model_from_sdf_world('gazebo_world_test', @world)
        end

        def data_dir
            File.expand_path('data', File.dirname(__FILE__))
        end

        it "creates tasks with no name directly using the context's name" do
            assert(model_task = @model.find_task_by_name('gazebo::underwater::oil_rig'))
            assert_equal 'rock_gazebo::ModelTask', model_task.task_model.name
            assert(model_task = @model.find_task_by_name('gazebo::underwater::oil_rig::flat_fish'))
            assert_equal 'rock_gazebo::ModelTask', model_task.task_model.name
        end

        it "resolves tasks with implicit names that are attached to sensors" do
            sensor_task = @model.find_task_by_name(
                'gazebo::underwater::oil_rig::flat_fish::body::lidar'
            )
            assert_equal 'rock_gazebo::LaserScanTask', sensor_task.task_model.name
        end

        it "resolves tasks with explicit names that are attached to sensors" do
            sensor_task = @model.find_task_by_name(
                'gazebo::underwater::oil_rig::flat_fish::body::lidar::lidar'
            )
            assert_equal 'rock_gazebo::LaserScanTask', sensor_task.task_model.name
        end

        it "appends an explicit task name to the context's name" do
            assert(model_task = @model.find_task_by_name("gazebo::underwater::oil_rig::flat_fish::gps"))
            assert_equal "rock_gazebo::GPSTask", model_task.task_model.name
        end

        it "allows to override the task's periodicity" do
            model = RockGazebo.orogen_model_from_sdf_world(
                'gazebo_world_test', @world, period: 0.001
            )

            %w[gazebo::underwater::oil_rig::flat_fish gazebo::underwater::oil_rig]
                .each do |task_name|
                    task = model.find_task_by_name(task_name)
                    assert_equal 0.001, task.period
                end
        end
    end
end
