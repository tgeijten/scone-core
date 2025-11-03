/*
** StorageIo.cpp
**
** Copyright (C) Thomas Geijtenbeek and contributors. All rights reserved.
**
** This file is part of SCONE. For more information, see http://scone.software.
*/

#include "StorageIo.h"

#include "xo/string/string_tools.h"
#include "xo/filesystem/path.h"
#include "xo/filesystem/filesystem.h"
#include "xo/numerical/constants.h"
#include <sstream>
#include <fstream>
#include "xo/utility/hash.h"
#include "Log.h"

#ifdef XO_COMP_MSVC
#pragma warning( disable: 4996 )
#endif

namespace scone
{
	constexpr double interval_epsilon = 1e-6;

	size_t CountFrames( const Storage<Real, TimeInSeconds>& storage, TimeInSeconds min_interval )
	{
		if ( min_interval == 0.0 )
			return storage.GetFrameCount();

		size_t frames = 0;
		auto prev_time = xo::constantsd::lowest();
		for ( auto& frame : storage.GetData() ) {
			auto t = frame->GetTime();
			if ( xo::greater_than_or_equal( t - prev_time, min_interval, interval_epsilon ) ) {
				++frames;
				prev_time = t;
			}
		}
		return frames;
	}

	void WriteStorageLabels( const Storage<Real, TimeInSeconds>& storage, std::ostream& str, const String& time_label )
	{
		str << time_label;
		for ( const String& label : storage.GetLabels() )
			str << "\t" << label;
		str << "\n";
	}

	void WriteStorageTxt( const Storage<Real, TimeInSeconds>& storage, std::ostream& str, const String& time_label, TimeInSeconds min_interval )
	{
		WriteStorageLabels( storage, str, time_label );

		size_t frame_count = 0;
		auto prev_time = xo::constantsd::lowest();
		for ( auto& frame : storage.GetData() )
		{
			auto t = frame->GetTime();
			if ( xo::greater_than_or_equal( t - prev_time, min_interval, interval_epsilon ) )
			{
				str << frame->GetTime();
				for ( size_t idx = 0; idx < storage.GetChannelCount(); ++idx )
					str << "\t" << ( *frame )[idx];
				str << "\n";
				prev_time = t;
				++frame_count;
			}
		}
	}

	void WriteStorageTxt( const Storage<Real, TimeInSeconds>& storage, std::FILE* f, const String& time_label, TimeInSeconds min_interval )
	{
		fprintf( f, "%s", time_label.c_str() );
		for ( const String& label : storage.GetLabels() )
			fprintf( f, "\t%s", label.c_str() );
		fprintf( f, "\n" );

		size_t frame_count = 0;
		auto prev_time = xo::constantsd::lowest();
		for ( auto& frame : storage.GetData() )
		{
			auto t = frame->GetTime();
			if ( xo::greater_than_or_equal( t - prev_time, min_interval, interval_epsilon ) ) {
				fprintf( f, "%g", frame->GetTime() );
				for ( size_t idx = 0; idx < storage.GetChannelCount(); ++idx )
					fprintf( f, "\t%g", ( *frame )[idx] );
				fprintf( f, "\n" );
				prev_time = t;
				++frame_count;
			}
		}
	}

	template< typename T >
	void WriteStorageBin( const Storage<Real, TimeInSeconds>& storage, std::ostream& str, const String& time_label, TimeInSeconds min_interval )
	{
		WriteStorageLabels( storage, str, time_label );

		size_t frame_count = 0;
		auto prev_time = xo::constantsd::lowest();
		for ( auto& frame : storage.GetData() ) {
			Real t = frame->GetTime();
			if ( xo::greater_than_or_equal( t - prev_time, min_interval, interval_epsilon ) ) {
				T tt = static_cast<T>( frame->GetTime() );
				str.write( reinterpret_cast<char*>( &tt ), sizeof( T ) );
				for ( size_t idx = 0; idx < storage.GetChannelCount(); ++idx ) {
					T vt = static_cast<T>( ( *frame )[idx] );
					str.write( reinterpret_cast<char*>( &vt ), sizeof( T ) );
				}
				prev_time = t;
				++frame_count;
			}
		}
	}

	void WriteStorageSto( const Storage<Real, TimeInSeconds>& storage, std::ostream& str, const String& name, TimeInSeconds min_interval )
	{
		// write header
		str << name << std::endl;
		str << "version=1" << std::endl;
		str << "nRows=" << CountFrames( storage, min_interval ) << std::endl;
		str << "nColumns=" << storage.GetChannelCount() + 1 << std::endl;
		str << "inDegrees=no" << std::endl;
		str << "endheader" << std::endl;

		// write data
		WriteStorageTxt( storage, str, "time", min_interval );
	}

	void WriteStorageSto( const Storage<Real, TimeInSeconds>& storage, std::FILE* f, const String& name, TimeInSeconds min_interval )
	{
		fprintf( f, "%s\nversion=1\nnRows=%zd\nnColumns=%zd\ninDegrees=no\nendheader\n",
			name.c_str(), CountFrames( storage, min_interval ), storage.GetChannelCount() + 1 );
		WriteStorageTxt( storage, f, "time", min_interval );
	}

	void WriteStorageTxt( const Storage<Real, TimeInSeconds>& storage, const xo::path& file, const String& time_label, TimeInSeconds min_interval )
	{
#ifdef XO_COMP_MSVC
		FILE* f = fopen( file.c_str(), "w" );
		SCONE_ERROR_IF( !f, "Could not open " + file.str() );
		WriteStorageTxt( storage, f, time_label, min_interval );
		fclose( f );
#else
		std::ofstream ofs( file.str() );
		SCONE_ASSERT_MSG( ofs.good(), "Could not open " + file.str() );
		WriteStorageTxt( storage, ofs, time_label, min_interval );
#endif
	}

	void WriteStorageSto( const Storage<Real, TimeInSeconds>& storage, const xo::path& file, const String& name, TimeInSeconds min_interval )
	{
#ifdef XO_COMP_MSVC
		FILE* f = fopen( file.c_str(), "w" );
		SCONE_ERROR_IF( !f, "Could not open " + file.str() );
		WriteStorageSto( storage, f, name, min_interval );
		fclose( f );
#else
		std::ofstream ofs( file.str() );
		SCONE_ASSERT_MSG( ofs.good(), "Could not open " + file.str() );
		WriteStorageSto( storage, ofs, name, min_interval );
#endif
	}

	void SCONE_API WriteStorageStob( const Storage< Real, TimeInSeconds >& storage, const xo::path& file, const String& name, TimeInSeconds min_interval )
	{
		std::ofstream str( file.str(), std::ios::binary );
		SCONE_ASSERT_MSG( str.good(), "Could not open " + file.str() );
		str << name << "\n";
		str << "version=1\n";
		str << "endheader\n";
		WriteStorageBin<float>( storage, str, "time", min_interval );
	}

	void ReadStorageTxt( Storage<Real, TimeInSeconds>& storage, xo::char_stream& str )
	{
		storage.Clear();

		// read time label
		String dummy;
		str >> dummy;
		//SCONE_ERROR_IF( dummy != "time", "First column should be labeled 'time'" );

		String header = str.get_line();
		SCONE_ERROR_IF( str.fail(), "Error reading file labels" );
		auto labels = xo::split_str( header, "\t " );

		// add channels to storage
		for ( auto& s : labels )
			storage.AddChannel( s );

		// read the data
		while ( str.good() )
		{
			double time;
			str >> time;

			if ( !str.good() )
				return; // stop if timestamp could not be read;

			auto& frame = storage.AddFrame( time );
			for ( size_t i = 0; i < storage.GetChannelCount(); ++i )
			{
				double value = 0.0;
				str >> value;
				frame[i] = value;
			}
		}
	}

	void ReadStorageBin( Storage<Real, TimeInSeconds>& storage, std::istream& str )
	{
		storage.Clear();

		std::string column_names;
		std::getline( str, column_names );
		SCONE_ERROR_IF( str.fail(), "Error reading file labels" );
		auto labels = xo::split_str( column_names, "\t " );
		for ( index_t i = 1; i < labels.size(); ++i )
			storage.AddChannel( labels[i] );

		// read the data
		auto columns = storage.GetChannelCount() + 1;
		std::vector<float> data( columns );
		while ( str.good() )
		{
			str.read( reinterpret_cast<char*>( data.data() ), columns * sizeof( float ) );
			if ( !str.good() )
				break;

			auto& frame = storage.AddFrame( data[0] );
			for ( size_t i = 0; i < storage.GetChannelCount(); ++i )
				frame[i] = data[i + 1];
		}
	}

	void ReadStorageSto( Storage<Real, TimeInSeconds>& storage, xo::char_stream& str )
	{
		// skip the header since we don't need it
		String s;
		while ( s != "endheader" && str.good() )
			str >> s;

		if ( str.good() )
			ReadStorageTxt( storage, str ); // read as txt once we have found the header
	}

	void ReadStorageTxt( Storage<Real, TimeInSeconds>& storage, const xo::path& file )
	{
		auto str = xo::char_stream( xo::load_string( file ) );
		SCONE_ERROR_IF( !str.good(), "Could not open " + file.str() );
		SCONE_TRY_RETHROW( ReadStorageTxt( storage, str ), "Error reading " + file.str() );
	}

	void ReadStorageSto( Storage<Real, TimeInSeconds>& storage, const xo::path& file )
	{
		auto str = xo::char_stream( xo::load_string( file ) );
		SCONE_ERROR_IF( !str.good(), "Could not open " + file.str() );
		SCONE_TRY_RETHROW( ReadStorageSto( storage, str ), "Error reading " + file.str() );
	}

	void ReadStorageStob( Storage< Real, TimeInSeconds >& storage, const xo::path& file )
	{
		auto str = std::ifstream( file.str(), std::ios::binary );
		SCONE_ERROR_IF( !str.good(), "Could not open " + file.str() );

		// read header
		std::string line;
		std::getline( str, line ); // name
		do {
			std::getline( str, line );
			SCONE_ERROR_IF( !str.good(), "Error reading " + file.str() );
			auto [key, value] = xo::make_key_value_str( line );
			if ( key == "version" && value != "1" )
				log::warning( "File " + file.str() + " version not supported (" + value + ")" );

		} while ( line != "endheader" );

		SCONE_TRY_RETHROW( ReadStorageBin( storage, str ), "Error reading " + file.str() );
	}

	void WriteStorage( const Storage< Real, TimeInSeconds >& storage, const xo::path& file, const String& name, TimeInSeconds min_interval )
	{
		switch ( xo::hash( file.extension_no_dot().str() ) )
		{
		case "txt"_hash: return WriteStorageTxt( storage, file, "time", min_interval );
		case "sto"_hash: return WriteStorageSto( storage, file, name, min_interval );
		case "stob"_hash: return WriteStorageStob( storage, file, name, min_interval );
		default: SCONE_ERROR( "Unsupported file format: " + file.str() );
		}
	}

	void ReadStorage( Storage<Real, TimeInSeconds>& storage, const xo::path& file )
	{
		switch ( xo::hash( file.extension_no_dot().str() ) )
		{
		case "txt"_hash: return ReadStorageTxt( storage, file );
		case "sto"_hash: return ReadStorageSto( storage, file );
		case "stob"_hash: return ReadStorageStob( storage, file );
		default: SCONE_ERROR( "Unsupported file format: " + file.str() );
		}
	}
}
