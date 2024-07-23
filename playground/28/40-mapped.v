// Benchmark "adder" written by ABC on Thu Jul 18 02:40:38 2024

module adder ( 
    \a[0] , \a[10] , \a[11] , \a[12] , \a[13] , \a[14] , \a[15] , \a[16] ,
    \a[17] , \a[18] , \a[19] , \a[1] , \a[20] , \a[21] , \a[22] , \a[23] ,
    \a[24] , \a[25] , \a[26] , \a[27] , \a[28] , \a[29] , \a[2] , \a[30] ,
    \a[31] , \a[3] , \a[4] , \a[5] , \a[6] , \a[7] , \a[8] , \a[9] ,
    \b[0] , \b[10] , \b[11] , \b[12] , \b[13] , \b[14] , \b[15] , \b[16] ,
    \b[17] , \b[18] , \b[19] , \b[1] , \b[20] , \b[21] , \b[22] , \b[23] ,
    \b[24] , \b[25] , \b[26] , \b[27] , \b[28] , \b[29] , \b[2] , \b[30] ,
    \b[3] , \b[4] , \b[5] , \b[6] , \b[7] , \b[8] , \b[9] ,
    \s[0] , \s[10] , \s[11] , \s[12] , \s[13] , \s[14] , \s[15] , \s[16] ,
    \s[17] , \s[18] , \s[19] , \s[1] , \s[20] , \s[21] , \s[22] , \s[23] ,
    \s[24] , \s[25] , \s[26] , \s[27] , \s[28] , \s[29] , \s[2] , \s[30] ,
    \s[31] , \s[3] , \s[4] , \s[5] , \s[6] , \s[7] , \s[8] , \s[9]   );
  input  \a[0] , \a[10] , \a[11] , \a[12] , \a[13] , \a[14] , \a[15] ,
    \a[16] , \a[17] , \a[18] , \a[19] , \a[1] , \a[20] , \a[21] , \a[22] ,
    \a[23] , \a[24] , \a[25] , \a[26] , \a[27] , \a[28] , \a[29] , \a[2] ,
    \a[30] , \a[31] , \a[3] , \a[4] , \a[5] , \a[6] , \a[7] , \a[8] ,
    \a[9] , \b[0] , \b[10] , \b[11] , \b[12] , \b[13] , \b[14] , \b[15] ,
    \b[16] , \b[17] , \b[18] , \b[19] , \b[1] , \b[20] , \b[21] , \b[22] ,
    \b[23] , \b[24] , \b[25] , \b[26] , \b[27] , \b[28] , \b[29] , \b[2] ,
    \b[30] , \b[3] , \b[4] , \b[5] , \b[6] , \b[7] , \b[8] , \b[9] ;
  output \s[0] , \s[10] , \s[11] , \s[12] , \s[13] , \s[14] , \s[15] ,
    \s[16] , \s[17] , \s[18] , \s[19] , \s[1] , \s[20] , \s[21] , \s[22] ,
    \s[23] , \s[24] , \s[25] , \s[26] , \s[27] , \s[28] , \s[29] , \s[2] ,
    \s[30] , \s[31] , \s[3] , \s[4] , \s[5] , \s[6] , \s[7] , \s[8] ,
    \s[9] ;
  wire new_n97, new_n98, new_n99, new_n100, new_n101, new_n102, new_n103,
    new_n104, new_n105, new_n106, new_n107, new_n108, new_n109, new_n110,
    new_n111, new_n112, new_n113, new_n114, new_n115, new_n116, new_n117,
    new_n118, new_n119, new_n120, new_n121, new_n122, new_n123, new_n124,
    new_n125, new_n126, new_n127, new_n128, new_n129, new_n130, new_n131,
    new_n132, new_n133, new_n134, new_n136, new_n137, new_n138, new_n139,
    new_n140, new_n141, new_n142, new_n143, new_n144, new_n145, new_n146,
    new_n147, new_n148, new_n149, new_n151, new_n152, new_n153, new_n154,
    new_n155, new_n157, new_n158, new_n159, new_n160, new_n161, new_n162,
    new_n163, new_n164, new_n165, new_n166, new_n167, new_n168, new_n169,
    new_n171, new_n172, new_n173, new_n174, new_n175, new_n177, new_n178,
    new_n179, new_n180, new_n182, new_n183, new_n184, new_n185, new_n186,
    new_n187, new_n189, new_n190, new_n191, new_n192, new_n193, new_n194,
    new_n195, new_n197, new_n198, new_n199, new_n200, new_n201, new_n202,
    new_n203, new_n204, new_n206, new_n207, new_n208, new_n209, new_n210,
    new_n211, new_n212, new_n213, new_n216, new_n217, new_n218, new_n219,
    new_n220, new_n221, new_n222, new_n223, new_n225, new_n226, new_n227,
    new_n228, new_n229, new_n230, new_n231, new_n232, new_n234, new_n235,
    new_n236, new_n237, new_n238, new_n239, new_n240, new_n242, new_n243,
    new_n244, new_n245, new_n246, new_n247, new_n248, new_n249, new_n250,
    new_n251, new_n252, new_n254, new_n255, new_n256, new_n257, new_n258,
    new_n260, new_n261, new_n262, new_n263, new_n264, new_n265, new_n266,
    new_n267, new_n268, new_n269, new_n271, new_n272, new_n273, new_n274,
    new_n275, new_n276, new_n277, new_n278, new_n280, new_n281, new_n282,
    new_n283, new_n284, new_n285, new_n286, new_n287, new_n288, new_n289,
    new_n290, new_n291, new_n293, new_n294, new_n295, new_n296, new_n297,
    new_n298, new_n299, new_n300, new_n301, new_n302, new_n303, new_n305,
    new_n306, new_n307, new_n308, new_n309, new_n310, new_n311, new_n312,
    new_n313, new_n316, new_n317, new_n318, new_n319, new_n320, new_n321,
    new_n322, new_n323, new_n324, new_n325, new_n327, new_n328, new_n329,
    new_n330, new_n331, new_n332, new_n333, new_n335, new_n337, new_n340,
    new_n341, new_n342, new_n343, new_n345, new_n347, new_n348, new_n349,
    new_n351;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor042aa1n04x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  inv000aa1d42x5               g002(.a(new_n97), .o1(new_n98));
  nand42aa1n02x5               g003(.a(\b[1] ), .b(\a[2] ), .o1(new_n99));
  oai012aa1n02x5               g004(.a(new_n99), .b(\b[2] ), .c(\a[3] ), .o1(new_n100));
  and002aa1n02x5               g005(.a(\b[3] ), .b(\a[4] ), .o(new_n101));
  inv000aa1d42x5               g006(.a(\a[4] ), .o1(new_n102));
  inv000aa1d42x5               g007(.a(\b[3] ), .o1(new_n103));
  aoi022aa1n02x5               g008(.a(new_n103), .b(new_n102), .c(\a[3] ), .d(\b[2] ), .o1(new_n104));
  nanp02aa1n02x5               g009(.a(\b[0] ), .b(\a[1] ), .o1(new_n105));
  nor002aa1n02x5               g010(.a(\b[1] ), .b(\a[2] ), .o1(new_n106));
  nona22aa1n06x5               g011(.a(new_n99), .b(new_n106), .c(new_n105), .out0(new_n107));
  nona23aa1n09x5               g012(.a(new_n107), .b(new_n104), .c(new_n100), .d(new_n101), .out0(new_n108));
  aoi112aa1n02x5               g013(.a(\b[2] ), .b(\a[3] ), .c(\a[4] ), .d(\b[3] ), .o1(new_n109));
  aoi012aa1n02x5               g014(.a(new_n109), .b(new_n102), .c(new_n103), .o1(new_n110));
  nanp02aa1n02x5               g015(.a(new_n108), .b(new_n110), .o1(new_n111));
  tech160nm_fixorc02aa1n03p5x5 g016(.a(\a[5] ), .b(\b[4] ), .out0(new_n112));
  nand42aa1n02x5               g017(.a(\b[5] ), .b(\a[6] ), .o1(new_n113));
  inv000aa1d42x5               g018(.a(\a[6] ), .o1(new_n114));
  inv000aa1d42x5               g019(.a(\b[5] ), .o1(new_n115));
  nand42aa1n03x5               g020(.a(new_n115), .b(new_n114), .o1(new_n116));
  xorc02aa1n12x5               g021(.a(\a[7] ), .b(\b[6] ), .out0(new_n117));
  nand22aa1n04x5               g022(.a(\b[7] ), .b(\a[8] ), .o1(new_n118));
  nor042aa1n04x5               g023(.a(\b[7] ), .b(\a[8] ), .o1(new_n119));
  norb02aa1n06x5               g024(.a(new_n118), .b(new_n119), .out0(new_n120));
  nanp02aa1n02x5               g025(.a(new_n117), .b(new_n120), .o1(new_n121));
  nano32aa1n03x7               g026(.a(new_n121), .b(new_n112), .c(new_n113), .d(new_n116), .out0(new_n122));
  nano22aa1n03x7               g027(.a(new_n119), .b(new_n113), .c(new_n118), .out0(new_n123));
  oai022aa1d18x5               g028(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n124));
  norp02aa1n02x5               g029(.a(\b[6] ), .b(\a[7] ), .o1(new_n125));
  tech160nm_fiao0012aa1n02p5x5 g030(.a(new_n119), .b(new_n125), .c(new_n118), .o(new_n126));
  aoi013aa1n09x5               g031(.a(new_n126), .b(new_n123), .c(new_n117), .d(new_n124), .o1(new_n127));
  inv000aa1d42x5               g032(.a(new_n127), .o1(new_n128));
  tech160nm_fixorc02aa1n03p5x5 g033(.a(\a[9] ), .b(\b[8] ), .out0(new_n129));
  aoai13aa1n02x5               g034(.a(new_n129), .b(new_n128), .c(new_n111), .d(new_n122), .o1(new_n130));
  nor022aa1n06x5               g035(.a(\b[9] ), .b(\a[10] ), .o1(new_n131));
  nand02aa1n06x5               g036(.a(\b[9] ), .b(\a[10] ), .o1(new_n132));
  nanb02aa1d24x5               g037(.a(new_n131), .b(new_n132), .out0(new_n133));
  inv000aa1d42x5               g038(.a(new_n133), .o1(new_n134));
  xnbna2aa1n03x5               g039(.a(new_n134), .b(new_n130), .c(new_n98), .out0(\s[10] ));
  inv040aa1d32x5               g040(.a(\a[11] ), .o1(new_n136));
  inv000aa1d42x5               g041(.a(\b[10] ), .o1(new_n137));
  nanp02aa1n04x5               g042(.a(new_n137), .b(new_n136), .o1(new_n138));
  nanp02aa1n02x5               g043(.a(\b[10] ), .b(\a[11] ), .o1(new_n139));
  nanp02aa1n02x5               g044(.a(new_n138), .b(new_n139), .o1(new_n140));
  nanp02aa1n02x5               g045(.a(new_n116), .b(new_n113), .o1(new_n141));
  nanb02aa1n02x5               g046(.a(new_n119), .b(new_n118), .out0(new_n142));
  nona23aa1n02x4               g047(.a(new_n117), .b(new_n112), .c(new_n142), .d(new_n141), .out0(new_n143));
  aoai13aa1n06x5               g048(.a(new_n127), .b(new_n143), .c(new_n108), .d(new_n110), .o1(new_n144));
  nor002aa1n02x5               g049(.a(new_n131), .b(new_n97), .o1(new_n145));
  inv000aa1d42x5               g050(.a(new_n145), .o1(new_n146));
  aoai13aa1n02x5               g051(.a(new_n132), .b(new_n146), .c(new_n144), .d(new_n129), .o1(new_n147));
  nanp03aa1n02x5               g052(.a(new_n138), .b(new_n132), .c(new_n139), .o1(new_n148));
  tech160nm_fiao0012aa1n02p5x5 g053(.a(new_n148), .b(new_n130), .c(new_n145), .o(new_n149));
  aobi12aa1n02x5               g054(.a(new_n149), .b(new_n147), .c(new_n140), .out0(\s[11] ));
  aoai13aa1n02x5               g055(.a(new_n138), .b(new_n148), .c(new_n130), .d(new_n145), .o1(new_n151));
  nor002aa1n02x5               g056(.a(\b[11] ), .b(\a[12] ), .o1(new_n152));
  nanp02aa1n02x5               g057(.a(\b[11] ), .b(\a[12] ), .o1(new_n153));
  norb02aa1n03x5               g058(.a(new_n153), .b(new_n152), .out0(new_n154));
  aboi22aa1n03x5               g059(.a(new_n152), .b(new_n153), .c(new_n136), .d(new_n137), .out0(new_n155));
  aoi022aa1n02x5               g060(.a(new_n149), .b(new_n155), .c(new_n151), .d(new_n154), .o1(\s[12] ));
  nona23aa1d18x5               g061(.a(new_n154), .b(new_n129), .c(new_n133), .d(new_n140), .out0(new_n157));
  inv000aa1d42x5               g062(.a(new_n157), .o1(new_n158));
  aoai13aa1n02x5               g063(.a(new_n158), .b(new_n128), .c(new_n111), .d(new_n122), .o1(new_n159));
  nano22aa1n02x5               g064(.a(new_n152), .b(new_n139), .c(new_n153), .out0(new_n160));
  oai012aa1n02x5               g065(.a(new_n132), .b(\b[10] ), .c(\a[11] ), .o1(new_n161));
  oab012aa1n04x5               g066(.a(new_n161), .b(new_n97), .c(new_n131), .out0(new_n162));
  oaoi03aa1n09x5               g067(.a(\a[12] ), .b(\b[11] ), .c(new_n138), .o1(new_n163));
  inv000aa1d42x5               g068(.a(new_n163), .o1(new_n164));
  aob012aa1n09x5               g069(.a(new_n164), .b(new_n162), .c(new_n160), .out0(new_n165));
  inv000aa1d42x5               g070(.a(new_n165), .o1(new_n166));
  nor042aa1n06x5               g071(.a(\b[12] ), .b(\a[13] ), .o1(new_n167));
  nanp02aa1n02x5               g072(.a(\b[12] ), .b(\a[13] ), .o1(new_n168));
  norb02aa1n02x5               g073(.a(new_n168), .b(new_n167), .out0(new_n169));
  xnbna2aa1n03x5               g074(.a(new_n169), .b(new_n159), .c(new_n166), .out0(\s[13] ));
  inv000aa1d42x5               g075(.a(new_n167), .o1(new_n171));
  aoai13aa1n02x5               g076(.a(new_n169), .b(new_n165), .c(new_n144), .d(new_n158), .o1(new_n172));
  nor042aa1n02x5               g077(.a(\b[13] ), .b(\a[14] ), .o1(new_n173));
  nanp02aa1n02x5               g078(.a(\b[13] ), .b(\a[14] ), .o1(new_n174));
  norb02aa1n02x5               g079(.a(new_n174), .b(new_n173), .out0(new_n175));
  xnbna2aa1n03x5               g080(.a(new_n175), .b(new_n172), .c(new_n171), .out0(\s[14] ));
  nano23aa1d12x5               g081(.a(new_n167), .b(new_n173), .c(new_n174), .d(new_n168), .out0(new_n177));
  aoai13aa1n06x5               g082(.a(new_n177), .b(new_n165), .c(new_n144), .d(new_n158), .o1(new_n178));
  oai012aa1n02x5               g083(.a(new_n174), .b(new_n173), .c(new_n167), .o1(new_n179));
  xorc02aa1n12x5               g084(.a(\a[15] ), .b(\b[14] ), .out0(new_n180));
  xnbna2aa1n03x5               g085(.a(new_n180), .b(new_n178), .c(new_n179), .out0(\s[15] ));
  aob012aa1n03x5               g086(.a(new_n180), .b(new_n178), .c(new_n179), .out0(new_n182));
  orn002aa1n02x5               g087(.a(\a[15] ), .b(\b[14] ), .o(new_n183));
  inv000aa1d42x5               g088(.a(new_n180), .o1(new_n184));
  aoai13aa1n02x5               g089(.a(new_n183), .b(new_n184), .c(new_n178), .d(new_n179), .o1(new_n185));
  xorc02aa1n02x5               g090(.a(\a[16] ), .b(\b[15] ), .out0(new_n186));
  norb02aa1n02x5               g091(.a(new_n183), .b(new_n186), .out0(new_n187));
  aoi022aa1n03x5               g092(.a(new_n185), .b(new_n186), .c(new_n182), .d(new_n187), .o1(\s[16] ));
  nano32aa1d12x5               g093(.a(new_n157), .b(new_n186), .c(new_n177), .d(new_n180), .out0(new_n189));
  aoai13aa1n06x5               g094(.a(new_n189), .b(new_n128), .c(new_n111), .d(new_n122), .o1(new_n190));
  nanp02aa1n02x5               g095(.a(new_n186), .b(new_n180), .o1(new_n191));
  aoai13aa1n06x5               g096(.a(new_n177), .b(new_n163), .c(new_n162), .d(new_n160), .o1(new_n192));
  aoi012aa1n02x5               g097(.a(new_n191), .b(new_n192), .c(new_n179), .o1(new_n193));
  tech160nm_fioaoi03aa1n02p5x5 g098(.a(\a[16] ), .b(\b[15] ), .c(new_n183), .o1(new_n194));
  nona22aa1n09x5               g099(.a(new_n190), .b(new_n193), .c(new_n194), .out0(new_n195));
  xorb03aa1n02x5               g100(.a(new_n195), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  aoi022aa1n02x5               g101(.a(\b[17] ), .b(\a[18] ), .c(\a[17] ), .d(\b[16] ), .o1(new_n197));
  oa0012aa1n02x5               g102(.a(new_n197), .b(\b[17] ), .c(\a[18] ), .o(new_n198));
  nor042aa1n03x5               g103(.a(\b[16] ), .b(\a[17] ), .o1(new_n199));
  inv000aa1n02x5               g104(.a(new_n194), .o1(new_n200));
  nona23aa1n02x4               g105(.a(new_n190), .b(new_n200), .c(new_n193), .d(new_n199), .out0(new_n201));
  xnrc02aa1n02x5               g106(.a(\b[17] ), .b(\a[18] ), .out0(new_n202));
  inv000aa1d42x5               g107(.a(\a[17] ), .o1(new_n203));
  oaib12aa1n02x5               g108(.a(new_n201), .b(new_n203), .c(\b[16] ), .out0(new_n204));
  aoi022aa1n02x5               g109(.a(new_n204), .b(new_n202), .c(new_n198), .d(new_n201), .o1(\s[18] ));
  aoai13aa1n06x5               g110(.a(new_n200), .b(new_n191), .c(new_n192), .d(new_n179), .o1(new_n206));
  inv000aa1d42x5               g111(.a(\a[18] ), .o1(new_n207));
  xroi22aa1d06x4               g112(.a(new_n203), .b(\b[16] ), .c(new_n207), .d(\b[17] ), .out0(new_n208));
  aoai13aa1n06x5               g113(.a(new_n208), .b(new_n206), .c(new_n144), .d(new_n189), .o1(new_n209));
  inv000aa1d42x5               g114(.a(\b[17] ), .o1(new_n210));
  oao003aa1n09x5               g115(.a(new_n207), .b(new_n210), .c(new_n199), .carry(new_n211));
  inv000aa1d42x5               g116(.a(new_n211), .o1(new_n212));
  xorc02aa1n12x5               g117(.a(\a[19] ), .b(\b[18] ), .out0(new_n213));
  xnbna2aa1n03x5               g118(.a(new_n213), .b(new_n209), .c(new_n212), .out0(\s[19] ));
  xnrc02aa1n02x5               g119(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  aoai13aa1n03x5               g120(.a(new_n213), .b(new_n211), .c(new_n195), .d(new_n208), .o1(new_n216));
  inv000aa1d42x5               g121(.a(\a[19] ), .o1(new_n217));
  inv000aa1d42x5               g122(.a(\b[18] ), .o1(new_n218));
  nanp02aa1n02x5               g123(.a(new_n218), .b(new_n217), .o1(new_n219));
  inv000aa1d42x5               g124(.a(new_n213), .o1(new_n220));
  aoai13aa1n02x5               g125(.a(new_n219), .b(new_n220), .c(new_n209), .d(new_n212), .o1(new_n221));
  xorc02aa1n02x5               g126(.a(\a[20] ), .b(\b[19] ), .out0(new_n222));
  norb02aa1n02x5               g127(.a(new_n219), .b(new_n222), .out0(new_n223));
  aoi022aa1n03x5               g128(.a(new_n221), .b(new_n222), .c(new_n216), .d(new_n223), .o1(\s[20] ));
  nand23aa1n04x5               g129(.a(new_n208), .b(new_n213), .c(new_n222), .o1(new_n225));
  inv000aa1d42x5               g130(.a(new_n225), .o1(new_n226));
  aoai13aa1n03x5               g131(.a(new_n226), .b(new_n206), .c(new_n144), .d(new_n189), .o1(new_n227));
  oaib12aa1n09x5               g132(.a(new_n211), .b(new_n218), .c(\a[19] ), .out0(new_n228));
  oa0022aa1n02x5               g133(.a(\a[20] ), .b(\b[19] ), .c(\a[19] ), .d(\b[18] ), .o(new_n229));
  aoi022aa1n06x5               g134(.a(new_n228), .b(new_n229), .c(\b[19] ), .d(\a[20] ), .o1(new_n230));
  inv000aa1d42x5               g135(.a(new_n230), .o1(new_n231));
  xorc02aa1n12x5               g136(.a(\a[21] ), .b(\b[20] ), .out0(new_n232));
  xnbna2aa1n03x5               g137(.a(new_n232), .b(new_n227), .c(new_n231), .out0(\s[21] ));
  aoai13aa1n04x5               g138(.a(new_n232), .b(new_n230), .c(new_n195), .d(new_n226), .o1(new_n234));
  nor042aa1n06x5               g139(.a(\b[20] ), .b(\a[21] ), .o1(new_n235));
  inv000aa1d42x5               g140(.a(new_n235), .o1(new_n236));
  inv000aa1d42x5               g141(.a(new_n232), .o1(new_n237));
  aoai13aa1n02x5               g142(.a(new_n236), .b(new_n237), .c(new_n227), .d(new_n231), .o1(new_n238));
  tech160nm_fixorc02aa1n03p5x5 g143(.a(\a[22] ), .b(\b[21] ), .out0(new_n239));
  norp02aa1n02x5               g144(.a(new_n239), .b(new_n235), .o1(new_n240));
  aoi022aa1n02x5               g145(.a(new_n238), .b(new_n239), .c(new_n234), .d(new_n240), .o1(\s[22] ));
  norp02aa1n02x5               g146(.a(\b[22] ), .b(\a[23] ), .o1(new_n242));
  and002aa1n03x5               g147(.a(\b[22] ), .b(\a[23] ), .o(new_n243));
  nor002aa1n04x5               g148(.a(new_n243), .b(new_n242), .o1(new_n244));
  nano22aa1n02x4               g149(.a(new_n225), .b(new_n232), .c(new_n239), .out0(new_n245));
  aoai13aa1n06x5               g150(.a(new_n245), .b(new_n206), .c(new_n144), .d(new_n189), .o1(new_n246));
  nanp02aa1n02x5               g151(.a(new_n228), .b(new_n229), .o1(new_n247));
  aoi012aa1n02x5               g152(.a(new_n235), .b(\a[20] ), .c(\b[19] ), .o1(new_n248));
  nand23aa1n06x5               g153(.a(new_n239), .b(new_n232), .c(new_n248), .o1(new_n249));
  inv000aa1d42x5               g154(.a(new_n249), .o1(new_n250));
  oaoi03aa1n02x5               g155(.a(\a[22] ), .b(\b[21] ), .c(new_n236), .o1(new_n251));
  aoi012aa1n02x5               g156(.a(new_n251), .b(new_n247), .c(new_n250), .o1(new_n252));
  xnbna2aa1n03x5               g157(.a(new_n244), .b(new_n246), .c(new_n252), .out0(\s[23] ));
  aoi013aa1n02x4               g158(.a(new_n243), .b(new_n246), .c(new_n252), .d(new_n244), .o1(new_n254));
  xorc02aa1n12x5               g159(.a(\a[24] ), .b(\b[23] ), .out0(new_n255));
  inv000aa1d42x5               g160(.a(new_n255), .o1(new_n256));
  nanp03aa1n02x5               g161(.a(new_n246), .b(new_n244), .c(new_n252), .o1(new_n257));
  nona22aa1n02x4               g162(.a(new_n257), .b(new_n255), .c(new_n243), .out0(new_n258));
  oai012aa1n02x5               g163(.a(new_n258), .b(new_n254), .c(new_n256), .o1(\s[24] ));
  and002aa1n02x5               g164(.a(new_n239), .b(new_n232), .o(new_n260));
  nand02aa1d06x5               g165(.a(new_n255), .b(new_n244), .o1(new_n261));
  inv040aa1n02x5               g166(.a(new_n261), .o1(new_n262));
  nano22aa1n02x4               g167(.a(new_n225), .b(new_n260), .c(new_n262), .out0(new_n263));
  aoai13aa1n06x5               g168(.a(new_n263), .b(new_n206), .c(new_n144), .d(new_n189), .o1(new_n264));
  oai022aa1n02x5               g169(.a(\a[23] ), .b(\b[22] ), .c(\b[23] ), .d(\a[24] ), .o1(new_n265));
  aob012aa1n02x5               g170(.a(new_n265), .b(\b[23] ), .c(\a[24] ), .out0(new_n266));
  oaib12aa1n12x5               g171(.a(new_n266), .b(new_n261), .c(new_n251), .out0(new_n267));
  aoi013aa1n09x5               g172(.a(new_n267), .b(new_n247), .c(new_n250), .d(new_n262), .o1(new_n268));
  xorc02aa1n12x5               g173(.a(\a[25] ), .b(\b[24] ), .out0(new_n269));
  xnbna2aa1n03x5               g174(.a(new_n269), .b(new_n264), .c(new_n268), .out0(\s[25] ));
  inv000aa1d42x5               g175(.a(new_n268), .o1(new_n271));
  aoai13aa1n03x5               g176(.a(new_n269), .b(new_n271), .c(new_n195), .d(new_n263), .o1(new_n272));
  nor042aa1n03x5               g177(.a(\b[24] ), .b(\a[25] ), .o1(new_n273));
  inv000aa1n03x5               g178(.a(new_n273), .o1(new_n274));
  inv000aa1d42x5               g179(.a(new_n269), .o1(new_n275));
  aoai13aa1n02x5               g180(.a(new_n274), .b(new_n275), .c(new_n264), .d(new_n268), .o1(new_n276));
  xorc02aa1n02x5               g181(.a(\a[26] ), .b(\b[25] ), .out0(new_n277));
  norp02aa1n02x5               g182(.a(new_n277), .b(new_n273), .o1(new_n278));
  aoi022aa1n02x5               g183(.a(new_n276), .b(new_n277), .c(new_n272), .d(new_n278), .o1(\s[26] ));
  norp02aa1n02x5               g184(.a(\b[26] ), .b(\a[27] ), .o1(new_n280));
  nand42aa1n06x5               g185(.a(\b[26] ), .b(\a[27] ), .o1(new_n281));
  norb02aa1n06x4               g186(.a(new_n281), .b(new_n280), .out0(new_n282));
  and002aa1n06x5               g187(.a(new_n277), .b(new_n269), .o(new_n283));
  nano32aa1n03x7               g188(.a(new_n225), .b(new_n283), .c(new_n260), .d(new_n262), .out0(new_n284));
  aoai13aa1n06x5               g189(.a(new_n284), .b(new_n206), .c(new_n144), .d(new_n189), .o1(new_n285));
  aoi112aa1n02x5               g190(.a(new_n249), .b(new_n261), .c(new_n228), .d(new_n229), .o1(new_n286));
  tech160nm_fioaoi03aa1n04x5   g191(.a(\a[26] ), .b(\b[25] ), .c(new_n274), .o1(new_n287));
  oaoi13aa1n06x5               g192(.a(new_n287), .b(new_n283), .c(new_n286), .d(new_n267), .o1(new_n288));
  inv000aa1d42x5               g193(.a(new_n283), .o1(new_n289));
  norb02aa1n02x5               g194(.a(new_n282), .b(new_n287), .out0(new_n290));
  oai112aa1n02x5               g195(.a(new_n285), .b(new_n290), .c(new_n289), .d(new_n268), .o1(new_n291));
  aoai13aa1n02x5               g196(.a(new_n291), .b(new_n282), .c(new_n288), .d(new_n285), .o1(\s[27] ));
  inv000aa1d42x5               g197(.a(new_n267), .o1(new_n293));
  nona22aa1n03x5               g198(.a(new_n247), .b(new_n249), .c(new_n261), .out0(new_n294));
  norp02aa1n02x5               g199(.a(new_n287), .b(new_n280), .o1(new_n295));
  aoai13aa1n02x5               g200(.a(new_n295), .b(new_n289), .c(new_n294), .d(new_n293), .o1(new_n296));
  nanb02aa1n02x5               g201(.a(new_n296), .b(new_n285), .out0(new_n297));
  inv000aa1d42x5               g202(.a(\a[28] ), .o1(new_n298));
  inv000aa1d42x5               g203(.a(\b[27] ), .o1(new_n299));
  aob012aa1n02x5               g204(.a(new_n281), .b(\b[27] ), .c(\a[28] ), .out0(new_n300));
  aoi012aa1n02x5               g205(.a(new_n300), .b(new_n298), .c(new_n299), .o1(new_n301));
  xnrc02aa1n02x5               g206(.a(\b[27] ), .b(\a[28] ), .out0(new_n302));
  aoai13aa1n02x5               g207(.a(new_n281), .b(new_n296), .c(new_n195), .d(new_n284), .o1(new_n303));
  aoi022aa1n03x5               g208(.a(new_n303), .b(new_n302), .c(new_n297), .d(new_n301), .o1(\s[28] ));
  inv000aa1d42x5               g209(.a(new_n287), .o1(new_n305));
  aoai13aa1n04x5               g210(.a(new_n305), .b(new_n289), .c(new_n294), .d(new_n293), .o1(new_n306));
  norb02aa1n06x5               g211(.a(new_n282), .b(new_n302), .out0(new_n307));
  aoai13aa1n02x5               g212(.a(new_n307), .b(new_n306), .c(new_n195), .d(new_n284), .o1(new_n308));
  inv000aa1d42x5               g213(.a(new_n307), .o1(new_n309));
  oaoi03aa1n02x5               g214(.a(new_n298), .b(new_n299), .c(new_n280), .o1(new_n310));
  aoai13aa1n03x5               g215(.a(new_n310), .b(new_n309), .c(new_n285), .d(new_n288), .o1(new_n311));
  tech160nm_fixorc02aa1n03p5x5 g216(.a(\a[29] ), .b(\b[28] ), .out0(new_n312));
  norb02aa1n02x5               g217(.a(new_n310), .b(new_n312), .out0(new_n313));
  aoi022aa1n03x5               g218(.a(new_n311), .b(new_n312), .c(new_n308), .d(new_n313), .o1(\s[29] ));
  xorb03aa1n02x5               g219(.a(new_n105), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n06x5               g220(.a(new_n302), .b(new_n312), .c(new_n282), .out0(new_n316));
  aoai13aa1n03x5               g221(.a(new_n316), .b(new_n306), .c(new_n195), .d(new_n284), .o1(new_n317));
  inv000aa1d42x5               g222(.a(new_n316), .o1(new_n318));
  oaoi03aa1n02x5               g223(.a(\a[29] ), .b(\b[28] ), .c(new_n310), .o1(new_n319));
  inv000aa1n03x5               g224(.a(new_n319), .o1(new_n320));
  aoai13aa1n03x5               g225(.a(new_n320), .b(new_n318), .c(new_n285), .d(new_n288), .o1(new_n321));
  xorc02aa1n02x5               g226(.a(\a[30] ), .b(\b[29] ), .out0(new_n322));
  aoi012aa1n02x5               g227(.a(new_n310), .b(\a[29] ), .c(\b[28] ), .o1(new_n323));
  oabi12aa1n02x5               g228(.a(new_n322), .b(\a[29] ), .c(\b[28] ), .out0(new_n324));
  norp02aa1n02x5               g229(.a(new_n324), .b(new_n323), .o1(new_n325));
  aoi022aa1n03x5               g230(.a(new_n321), .b(new_n322), .c(new_n317), .d(new_n325), .o1(\s[30] ));
  nano22aa1d24x5               g231(.a(new_n309), .b(new_n312), .c(new_n322), .out0(new_n327));
  aoai13aa1n02x5               g232(.a(new_n327), .b(new_n306), .c(new_n195), .d(new_n284), .o1(new_n328));
  inv000aa1d42x5               g233(.a(new_n327), .o1(new_n329));
  oao003aa1n02x5               g234(.a(\a[30] ), .b(\b[29] ), .c(new_n320), .carry(new_n330));
  aoai13aa1n03x5               g235(.a(new_n330), .b(new_n329), .c(new_n285), .d(new_n288), .o1(new_n331));
  xorc02aa1n02x5               g236(.a(\a[31] ), .b(\b[30] ), .out0(new_n332));
  norb02aa1n02x5               g237(.a(new_n330), .b(new_n332), .out0(new_n333));
  aoi022aa1n03x5               g238(.a(new_n331), .b(new_n332), .c(new_n328), .d(new_n333), .o1(\s[31] ));
  oai012aa1n02x5               g239(.a(new_n99), .b(new_n106), .c(new_n105), .o1(new_n335));
  xnrb03aa1n02x5               g240(.a(new_n335), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g241(.a(\a[3] ), .b(\b[2] ), .c(new_n335), .o1(new_n337));
  xorb03aa1n02x5               g242(.a(new_n337), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xnbna2aa1n03x5               g243(.a(new_n112), .b(new_n108), .c(new_n110), .out0(\s[5] ));
  nanp03aa1n02x5               g244(.a(new_n108), .b(new_n110), .c(new_n112), .o1(new_n340));
  and002aa1n02x5               g245(.a(\b[4] ), .b(\a[5] ), .o(new_n341));
  aboi22aa1n03x5               g246(.a(new_n341), .b(new_n340), .c(new_n113), .d(new_n116), .out0(new_n342));
  aoi022aa1n02x5               g247(.a(\b[5] ), .b(\a[6] ), .c(\a[5] ), .d(\b[4] ), .o1(new_n343));
  aoi013aa1n02x4               g248(.a(new_n342), .b(new_n340), .c(new_n116), .d(new_n343), .o1(\s[6] ));
  aoi022aa1n02x5               g249(.a(new_n340), .b(new_n343), .c(new_n115), .d(new_n114), .o1(new_n345));
  xnrc02aa1n02x5               g250(.a(new_n345), .b(new_n117), .out0(\s[7] ));
  nanb02aa1n02x5               g251(.a(new_n345), .b(new_n117), .out0(new_n347));
  oaoi03aa1n02x5               g252(.a(\a[7] ), .b(\b[6] ), .c(new_n345), .o1(new_n348));
  aoib12aa1n02x5               g253(.a(new_n125), .b(new_n118), .c(new_n119), .out0(new_n349));
  aoi022aa1n02x5               g254(.a(new_n347), .b(new_n349), .c(new_n348), .d(new_n120), .o1(\s[8] ));
  aoi112aa1n02x5               g255(.a(new_n128), .b(new_n129), .c(new_n111), .d(new_n122), .o1(new_n351));
  aoi012aa1n02x5               g256(.a(new_n351), .b(new_n144), .c(new_n129), .o1(\s[9] ));
endmodule


