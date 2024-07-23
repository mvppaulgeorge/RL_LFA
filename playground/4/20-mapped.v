// Benchmark "adder" written by ABC on Wed Jul 17 14:07:52 2024

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
    new_n132, new_n133, new_n134, new_n135, new_n137, new_n138, new_n139,
    new_n140, new_n141, new_n143, new_n144, new_n145, new_n146, new_n147,
    new_n148, new_n149, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n159, new_n160, new_n161, new_n162,
    new_n164, new_n165, new_n166, new_n167, new_n169, new_n170, new_n171,
    new_n172, new_n173, new_n174, new_n175, new_n176, new_n178, new_n179,
    new_n180, new_n181, new_n182, new_n183, new_n184, new_n186, new_n187,
    new_n188, new_n189, new_n190, new_n191, new_n192, new_n194, new_n195,
    new_n196, new_n197, new_n198, new_n200, new_n201, new_n202, new_n203,
    new_n204, new_n205, new_n206, new_n207, new_n208, new_n211, new_n212,
    new_n213, new_n214, new_n216, new_n217, new_n218, new_n219, new_n220,
    new_n221, new_n222, new_n223, new_n224, new_n225, new_n226, new_n228,
    new_n229, new_n230, new_n231, new_n232, new_n234, new_n235, new_n236,
    new_n237, new_n238, new_n239, new_n240, new_n241, new_n242, new_n243,
    new_n244, new_n245, new_n247, new_n248, new_n249, new_n250, new_n252,
    new_n253, new_n254, new_n255, new_n256, new_n257, new_n258, new_n259,
    new_n260, new_n261, new_n263, new_n264, new_n265, new_n266, new_n267,
    new_n268, new_n269, new_n270, new_n271, new_n272, new_n273, new_n275,
    new_n276, new_n277, new_n278, new_n279, new_n280, new_n281, new_n282,
    new_n283, new_n284, new_n285, new_n286, new_n287, new_n289, new_n290,
    new_n291, new_n292, new_n293, new_n294, new_n295, new_n297, new_n298,
    new_n299, new_n300, new_n301, new_n302, new_n303, new_n304, new_n305,
    new_n308, new_n309, new_n310, new_n311, new_n312, new_n313, new_n314,
    new_n315, new_n316, new_n317, new_n319, new_n320, new_n321, new_n322,
    new_n323, new_n324, new_n325, new_n328, new_n329, new_n330, new_n331,
    new_n332, new_n333, new_n336, new_n338, new_n339, new_n340, new_n342;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor002aa1n06x5               g001(.a(\b[9] ), .b(\a[10] ), .o1(new_n97));
  nand22aa1n12x5               g002(.a(\b[9] ), .b(\a[10] ), .o1(new_n98));
  inv000aa1d42x5               g003(.a(new_n98), .o1(new_n99));
  norp02aa1n12x5               g004(.a(\b[8] ), .b(\a[9] ), .o1(new_n100));
  nanp02aa1n02x5               g005(.a(\b[8] ), .b(\a[9] ), .o1(new_n101));
  orn002aa1n02x7               g006(.a(\a[2] ), .b(\b[1] ), .o(new_n102));
  nand02aa1n03x5               g007(.a(\b[0] ), .b(\a[1] ), .o1(new_n103));
  aob012aa1n03x5               g008(.a(new_n103), .b(\b[1] ), .c(\a[2] ), .out0(new_n104));
  inv040aa1d32x5               g009(.a(\a[3] ), .o1(new_n105));
  inv000aa1d42x5               g010(.a(\b[2] ), .o1(new_n106));
  nand22aa1n03x5               g011(.a(new_n106), .b(new_n105), .o1(new_n107));
  nanp02aa1n02x5               g012(.a(\b[2] ), .b(\a[3] ), .o1(new_n108));
  nanp02aa1n04x5               g013(.a(new_n107), .b(new_n108), .o1(new_n109));
  inv000aa1d42x5               g014(.a(\a[4] ), .o1(new_n110));
  aboi22aa1n03x5               g015(.a(\b[3] ), .b(new_n110), .c(new_n105), .d(new_n106), .out0(new_n111));
  aoai13aa1n03x5               g016(.a(new_n111), .b(new_n109), .c(new_n104), .d(new_n102), .o1(new_n112));
  nanp02aa1n02x5               g017(.a(\b[4] ), .b(\a[5] ), .o1(new_n113));
  norp02aa1n02x5               g018(.a(\b[4] ), .b(\a[5] ), .o1(new_n114));
  aoi012aa1n02x5               g019(.a(new_n114), .b(\a[6] ), .c(\b[5] ), .o1(new_n115));
  inv000aa1d42x5               g020(.a(\a[8] ), .o1(new_n116));
  inv000aa1d42x5               g021(.a(\b[7] ), .o1(new_n117));
  aoi022aa1n02x5               g022(.a(new_n117), .b(new_n116), .c(\a[4] ), .d(\b[3] ), .o1(new_n118));
  nanp02aa1n02x5               g023(.a(\b[7] ), .b(\a[8] ), .o1(new_n119));
  nor042aa1n02x5               g024(.a(\b[6] ), .b(\a[7] ), .o1(new_n120));
  nand42aa1n02x5               g025(.a(\b[6] ), .b(\a[7] ), .o1(new_n121));
  nor042aa1n02x5               g026(.a(\b[5] ), .b(\a[6] ), .o1(new_n122));
  nona23aa1n03x5               g027(.a(new_n121), .b(new_n119), .c(new_n122), .d(new_n120), .out0(new_n123));
  nano32aa1n02x4               g028(.a(new_n123), .b(new_n118), .c(new_n115), .d(new_n113), .out0(new_n124));
  nand42aa1n02x5               g029(.a(new_n124), .b(new_n112), .o1(new_n125));
  nanp02aa1n02x5               g030(.a(\b[5] ), .b(\a[6] ), .o1(new_n126));
  nano22aa1n02x4               g031(.a(new_n120), .b(new_n126), .c(new_n121), .out0(new_n127));
  oai022aa1n02x5               g032(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n128));
  tech160nm_fixorc02aa1n04x5   g033(.a(\a[8] ), .b(\b[7] ), .out0(new_n129));
  oao003aa1n02x5               g034(.a(new_n116), .b(new_n117), .c(new_n120), .carry(new_n130));
  aoi013aa1n06x4               g035(.a(new_n130), .b(new_n127), .c(new_n129), .d(new_n128), .o1(new_n131));
  nanp02aa1n03x5               g036(.a(new_n125), .b(new_n131), .o1(new_n132));
  nanp02aa1n03x5               g037(.a(new_n132), .b(new_n101), .o1(new_n133));
  obai22aa1n02x7               g038(.a(new_n133), .b(new_n100), .c(new_n97), .d(new_n99), .out0(new_n134));
  nona32aa1n06x5               g039(.a(new_n133), .b(new_n100), .c(new_n99), .d(new_n97), .out0(new_n135));
  nanp02aa1n02x5               g040(.a(new_n134), .b(new_n135), .o1(\s[10] ));
  nanp02aa1n09x5               g041(.a(\b[10] ), .b(\a[11] ), .o1(new_n137));
  nor042aa1n06x5               g042(.a(\b[10] ), .b(\a[11] ), .o1(new_n138));
  aoi012aa1n02x5               g043(.a(new_n138), .b(\a[10] ), .c(\b[9] ), .o1(new_n139));
  inv000aa1n04x5               g044(.a(new_n138), .o1(new_n140));
  aoi022aa1n02x5               g045(.a(new_n135), .b(new_n98), .c(new_n140), .d(new_n137), .o1(new_n141));
  aoi013aa1n02x4               g046(.a(new_n141), .b(new_n139), .c(new_n137), .d(new_n135), .o1(\s[11] ));
  nand23aa1n03x5               g047(.a(new_n135), .b(new_n137), .c(new_n139), .o1(new_n143));
  nor042aa1d18x5               g048(.a(\b[11] ), .b(\a[12] ), .o1(new_n144));
  nanp02aa1n09x5               g049(.a(\b[11] ), .b(\a[12] ), .o1(new_n145));
  norb02aa1n06x4               g050(.a(new_n145), .b(new_n144), .out0(new_n146));
  inv000aa1d42x5               g051(.a(new_n144), .o1(new_n147));
  aoi012aa1n02x5               g052(.a(new_n138), .b(new_n147), .c(new_n145), .o1(new_n148));
  nanp02aa1n02x5               g053(.a(new_n143), .b(new_n140), .o1(new_n149));
  aoi022aa1n02x5               g054(.a(new_n149), .b(new_n146), .c(new_n143), .d(new_n148), .o1(\s[12] ));
  nona23aa1n03x5               g055(.a(new_n101), .b(new_n98), .c(new_n97), .d(new_n100), .out0(new_n151));
  nano32aa1n03x7               g056(.a(new_n151), .b(new_n146), .c(new_n140), .d(new_n137), .out0(new_n152));
  nanb03aa1n03x5               g057(.a(new_n144), .b(new_n145), .c(new_n137), .out0(new_n153));
  oai112aa1n03x5               g058(.a(new_n140), .b(new_n98), .c(new_n100), .d(new_n97), .o1(new_n154));
  tech160nm_fiaoi012aa1n03p5x5 g059(.a(new_n144), .b(new_n138), .c(new_n145), .o1(new_n155));
  tech160nm_fioai012aa1n04x5   g060(.a(new_n155), .b(new_n154), .c(new_n153), .o1(new_n156));
  nor002aa1d32x5               g061(.a(\b[12] ), .b(\a[13] ), .o1(new_n157));
  nand22aa1n04x5               g062(.a(\b[12] ), .b(\a[13] ), .o1(new_n158));
  nanb02aa1n02x5               g063(.a(new_n157), .b(new_n158), .out0(new_n159));
  inv000aa1d42x5               g064(.a(new_n159), .o1(new_n160));
  aoai13aa1n02x5               g065(.a(new_n160), .b(new_n156), .c(new_n132), .d(new_n152), .o1(new_n161));
  aoi112aa1n02x5               g066(.a(new_n160), .b(new_n156), .c(new_n132), .d(new_n152), .o1(new_n162));
  norb02aa1n02x5               g067(.a(new_n161), .b(new_n162), .out0(\s[13] ));
  inv000aa1d42x5               g068(.a(new_n157), .o1(new_n164));
  nor002aa1d32x5               g069(.a(\b[13] ), .b(\a[14] ), .o1(new_n165));
  nand22aa1n09x5               g070(.a(\b[13] ), .b(\a[14] ), .o1(new_n166));
  norb02aa1n02x5               g071(.a(new_n166), .b(new_n165), .out0(new_n167));
  xnbna2aa1n03x5               g072(.a(new_n167), .b(new_n161), .c(new_n164), .out0(\s[14] ));
  nona23aa1d18x5               g073(.a(new_n166), .b(new_n158), .c(new_n157), .d(new_n165), .out0(new_n169));
  inv000aa1d42x5               g074(.a(new_n169), .o1(new_n170));
  aoai13aa1n06x5               g075(.a(new_n170), .b(new_n156), .c(new_n132), .d(new_n152), .o1(new_n171));
  tech160nm_fiaoi012aa1n03p5x5 g076(.a(new_n165), .b(new_n157), .c(new_n166), .o1(new_n172));
  nor042aa1d18x5               g077(.a(\b[14] ), .b(\a[15] ), .o1(new_n173));
  nanp02aa1n02x5               g078(.a(\b[14] ), .b(\a[15] ), .o1(new_n174));
  nanb02aa1n02x5               g079(.a(new_n173), .b(new_n174), .out0(new_n175));
  inv000aa1d42x5               g080(.a(new_n175), .o1(new_n176));
  xnbna2aa1n03x5               g081(.a(new_n176), .b(new_n171), .c(new_n172), .out0(\s[15] ));
  aob012aa1n03x5               g082(.a(new_n176), .b(new_n171), .c(new_n172), .out0(new_n178));
  nor042aa1n03x5               g083(.a(\b[15] ), .b(\a[16] ), .o1(new_n179));
  nand22aa1n02x5               g084(.a(\b[15] ), .b(\a[16] ), .o1(new_n180));
  norb02aa1n02x5               g085(.a(new_n180), .b(new_n179), .out0(new_n181));
  aoib12aa1n02x5               g086(.a(new_n173), .b(new_n180), .c(new_n179), .out0(new_n182));
  inv000aa1d42x5               g087(.a(new_n173), .o1(new_n183));
  aoai13aa1n02x5               g088(.a(new_n183), .b(new_n175), .c(new_n171), .d(new_n172), .o1(new_n184));
  aoi022aa1n02x5               g089(.a(new_n184), .b(new_n181), .c(new_n178), .d(new_n182), .o1(\s[16] ));
  nona23aa1n09x5               g090(.a(new_n180), .b(new_n174), .c(new_n173), .d(new_n179), .out0(new_n186));
  nor042aa1n06x5               g091(.a(new_n186), .b(new_n169), .o1(new_n187));
  nand22aa1n09x5               g092(.a(new_n152), .b(new_n187), .o1(new_n188));
  aoi012aa1n02x5               g093(.a(new_n179), .b(new_n173), .c(new_n180), .o1(new_n189));
  oai012aa1n02x5               g094(.a(new_n189), .b(new_n186), .c(new_n172), .o1(new_n190));
  aoi012aa1n06x5               g095(.a(new_n190), .b(new_n156), .c(new_n187), .o1(new_n191));
  aoai13aa1n12x5               g096(.a(new_n191), .b(new_n188), .c(new_n125), .d(new_n131), .o1(new_n192));
  xorb03aa1n02x5               g097(.a(new_n192), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv030aa1d32x5               g098(.a(\a[17] ), .o1(new_n194));
  inv000aa1d42x5               g099(.a(\b[16] ), .o1(new_n195));
  nanp02aa1n02x5               g100(.a(new_n195), .b(new_n194), .o1(new_n196));
  oaib12aa1n02x5               g101(.a(new_n192), .b(new_n195), .c(\a[17] ), .out0(new_n197));
  xorc02aa1n02x5               g102(.a(\a[18] ), .b(\b[17] ), .out0(new_n198));
  xnbna2aa1n03x5               g103(.a(new_n198), .b(new_n197), .c(new_n196), .out0(\s[18] ));
  inv040aa1d32x5               g104(.a(\a[18] ), .o1(new_n200));
  xroi22aa1d06x4               g105(.a(new_n194), .b(\b[16] ), .c(new_n200), .d(\b[17] ), .out0(new_n201));
  norp02aa1n02x5               g106(.a(\b[17] ), .b(\a[18] ), .o1(new_n202));
  aoi112aa1n09x5               g107(.a(\b[16] ), .b(\a[17] ), .c(\a[18] ), .d(\b[17] ), .o1(new_n203));
  norp02aa1n02x5               g108(.a(new_n203), .b(new_n202), .o1(new_n204));
  inv000aa1d42x5               g109(.a(new_n204), .o1(new_n205));
  xorc02aa1n12x5               g110(.a(\a[19] ), .b(\b[18] ), .out0(new_n206));
  aoai13aa1n06x5               g111(.a(new_n206), .b(new_n205), .c(new_n192), .d(new_n201), .o1(new_n207));
  aoi112aa1n02x5               g112(.a(new_n206), .b(new_n205), .c(new_n192), .d(new_n201), .o1(new_n208));
  norb02aa1n02x5               g113(.a(new_n207), .b(new_n208), .out0(\s[19] ));
  xnrc02aa1n02x5               g114(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  tech160nm_fixorc02aa1n05x5   g115(.a(\a[20] ), .b(\b[19] ), .out0(new_n211));
  inv000aa1d42x5               g116(.a(\a[19] ), .o1(new_n212));
  aoib12aa1n02x5               g117(.a(new_n211), .b(new_n212), .c(\b[18] ), .out0(new_n213));
  oaib12aa1n02x5               g118(.a(new_n207), .b(\b[18] ), .c(new_n212), .out0(new_n214));
  aoi022aa1n02x5               g119(.a(new_n214), .b(new_n211), .c(new_n207), .d(new_n213), .o1(\s[20] ));
  inv030aa1d32x5               g120(.a(\a[20] ), .o1(new_n216));
  xroi22aa1d06x4               g121(.a(new_n212), .b(\b[18] ), .c(new_n216), .d(\b[19] ), .out0(new_n217));
  nand02aa1n03x5               g122(.a(new_n217), .b(new_n201), .o1(new_n218));
  inv000aa1d42x5               g123(.a(new_n218), .o1(new_n219));
  oai112aa1n03x5               g124(.a(new_n206), .b(new_n211), .c(new_n203), .d(new_n202), .o1(new_n220));
  aoi112aa1n03x5               g125(.a(\b[18] ), .b(\a[19] ), .c(\a[20] ), .d(\b[19] ), .o1(new_n221));
  aoib12aa1n02x5               g126(.a(new_n221), .b(new_n216), .c(\b[19] ), .out0(new_n222));
  nanp02aa1n02x5               g127(.a(new_n220), .b(new_n222), .o1(new_n223));
  xorc02aa1n02x5               g128(.a(\a[21] ), .b(\b[20] ), .out0(new_n224));
  aoai13aa1n06x5               g129(.a(new_n224), .b(new_n223), .c(new_n192), .d(new_n219), .o1(new_n225));
  aoi112aa1n02x5               g130(.a(new_n224), .b(new_n223), .c(new_n192), .d(new_n219), .o1(new_n226));
  norb02aa1n02x5               g131(.a(new_n225), .b(new_n226), .out0(\s[21] ));
  xorc02aa1n02x5               g132(.a(\a[22] ), .b(\b[21] ), .out0(new_n228));
  norp02aa1n02x5               g133(.a(\b[20] ), .b(\a[21] ), .o1(new_n229));
  norp02aa1n02x5               g134(.a(new_n228), .b(new_n229), .o1(new_n230));
  inv040aa1d30x5               g135(.a(\a[21] ), .o1(new_n231));
  oaib12aa1n02x5               g136(.a(new_n225), .b(\b[20] ), .c(new_n231), .out0(new_n232));
  aoi022aa1n02x5               g137(.a(new_n232), .b(new_n228), .c(new_n225), .d(new_n230), .o1(\s[22] ));
  inv040aa1d32x5               g138(.a(\a[22] ), .o1(new_n234));
  xroi22aa1d06x4               g139(.a(new_n231), .b(\b[20] ), .c(new_n234), .d(\b[21] ), .out0(new_n235));
  inv020aa1n02x5               g140(.a(new_n235), .o1(new_n236));
  nano22aa1n02x4               g141(.a(new_n236), .b(new_n201), .c(new_n217), .out0(new_n237));
  inv000aa1d42x5               g142(.a(\b[21] ), .o1(new_n238));
  oaoi03aa1n02x5               g143(.a(new_n234), .b(new_n238), .c(new_n229), .o1(new_n239));
  aoai13aa1n06x5               g144(.a(new_n239), .b(new_n236), .c(new_n220), .d(new_n222), .o1(new_n240));
  tech160nm_fixorc02aa1n03p5x5 g145(.a(\a[23] ), .b(\b[22] ), .out0(new_n241));
  aoai13aa1n06x5               g146(.a(new_n241), .b(new_n240), .c(new_n192), .d(new_n237), .o1(new_n242));
  nanb02aa1n02x5               g147(.a(new_n241), .b(new_n239), .out0(new_n243));
  aoi012aa1n02x5               g148(.a(new_n243), .b(new_n223), .c(new_n235), .o1(new_n244));
  aobi12aa1n02x5               g149(.a(new_n244), .b(new_n192), .c(new_n237), .out0(new_n245));
  norb02aa1n02x5               g150(.a(new_n242), .b(new_n245), .out0(\s[23] ));
  tech160nm_fixorc02aa1n03p5x5 g151(.a(\a[24] ), .b(\b[23] ), .out0(new_n247));
  norp02aa1n02x5               g152(.a(\b[22] ), .b(\a[23] ), .o1(new_n248));
  norp02aa1n02x5               g153(.a(new_n247), .b(new_n248), .o1(new_n249));
  oai012aa1n02x5               g154(.a(new_n242), .b(\b[22] ), .c(\a[23] ), .o1(new_n250));
  aoi022aa1n02x5               g155(.a(new_n250), .b(new_n247), .c(new_n242), .d(new_n249), .o1(\s[24] ));
  and002aa1n06x5               g156(.a(new_n247), .b(new_n241), .o(new_n252));
  nona23aa1n06x5               g157(.a(new_n192), .b(new_n252), .c(new_n236), .d(new_n218), .out0(new_n253));
  inv000aa1d42x5               g158(.a(\a[24] ), .o1(new_n254));
  inv000aa1d42x5               g159(.a(\b[23] ), .o1(new_n255));
  tech160nm_fioaoi03aa1n02p5x5 g160(.a(new_n254), .b(new_n255), .c(new_n248), .o1(new_n256));
  inv000aa1d42x5               g161(.a(new_n256), .o1(new_n257));
  aoi012aa1d18x5               g162(.a(new_n257), .b(new_n240), .c(new_n252), .o1(new_n258));
  nanp02aa1n02x5               g163(.a(new_n253), .b(new_n258), .o1(new_n259));
  xorc02aa1n12x5               g164(.a(\a[25] ), .b(\b[24] ), .out0(new_n260));
  aoi112aa1n02x5               g165(.a(new_n260), .b(new_n257), .c(new_n240), .d(new_n252), .o1(new_n261));
  aoi022aa1n02x5               g166(.a(new_n259), .b(new_n260), .c(new_n253), .d(new_n261), .o1(\s[25] ));
  inv000aa1d42x5               g167(.a(new_n258), .o1(new_n263));
  inv000aa1d42x5               g168(.a(new_n188), .o1(new_n264));
  aobi12aa1n02x5               g169(.a(new_n191), .b(new_n132), .c(new_n264), .out0(new_n265));
  nano32aa1n02x4               g170(.a(new_n265), .b(new_n252), .c(new_n219), .d(new_n235), .out0(new_n266));
  oai012aa1n02x5               g171(.a(new_n260), .b(new_n266), .c(new_n263), .o1(new_n267));
  xorc02aa1n02x5               g172(.a(\a[26] ), .b(\b[25] ), .out0(new_n268));
  nor042aa1n03x5               g173(.a(\b[24] ), .b(\a[25] ), .o1(new_n269));
  norp02aa1n02x5               g174(.a(new_n268), .b(new_n269), .o1(new_n270));
  inv000aa1d42x5               g175(.a(new_n269), .o1(new_n271));
  inv000aa1d42x5               g176(.a(new_n260), .o1(new_n272));
  aoai13aa1n04x5               g177(.a(new_n271), .b(new_n272), .c(new_n253), .d(new_n258), .o1(new_n273));
  aoi022aa1n03x5               g178(.a(new_n273), .b(new_n268), .c(new_n267), .d(new_n270), .o1(\s[26] ));
  and002aa1n02x5               g179(.a(new_n268), .b(new_n260), .o(new_n275));
  aoai13aa1n09x5               g180(.a(new_n275), .b(new_n257), .c(new_n240), .d(new_n252), .o1(new_n276));
  nano32aa1n03x7               g181(.a(new_n218), .b(new_n275), .c(new_n235), .d(new_n252), .out0(new_n277));
  oao003aa1n02x5               g182(.a(\a[26] ), .b(\b[25] ), .c(new_n271), .carry(new_n278));
  inv000aa1d42x5               g183(.a(new_n278), .o1(new_n279));
  tech160nm_fiaoi012aa1n05x5   g184(.a(new_n279), .b(new_n192), .c(new_n277), .o1(new_n280));
  norp02aa1n02x5               g185(.a(\b[26] ), .b(\a[27] ), .o1(new_n281));
  and002aa1n02x5               g186(.a(\b[26] ), .b(\a[27] ), .o(new_n282));
  norp02aa1n02x5               g187(.a(new_n282), .b(new_n281), .o1(new_n283));
  nand02aa1n04x5               g188(.a(new_n192), .b(new_n277), .o1(new_n284));
  inv040aa1n03x5               g189(.a(new_n283), .o1(new_n285));
  norb02aa1n02x5               g190(.a(new_n278), .b(new_n285), .out0(new_n286));
  nanp03aa1n02x5               g191(.a(new_n276), .b(new_n284), .c(new_n286), .o1(new_n287));
  aoai13aa1n02x5               g192(.a(new_n287), .b(new_n283), .c(new_n280), .d(new_n276), .o1(\s[27] ));
  inv000aa1d42x5               g193(.a(\b[26] ), .o1(new_n289));
  nanp03aa1n06x5               g194(.a(new_n276), .b(new_n284), .c(new_n278), .o1(new_n290));
  oaib12aa1n03x5               g195(.a(new_n290), .b(new_n289), .c(\a[27] ), .out0(new_n291));
  inv000aa1n06x5               g196(.a(new_n281), .o1(new_n292));
  aoai13aa1n02x7               g197(.a(new_n292), .b(new_n282), .c(new_n280), .d(new_n276), .o1(new_n293));
  xorc02aa1n02x5               g198(.a(\a[28] ), .b(\b[27] ), .out0(new_n294));
  norp02aa1n02x5               g199(.a(new_n294), .b(new_n281), .o1(new_n295));
  aoi022aa1n03x5               g200(.a(new_n293), .b(new_n294), .c(new_n291), .d(new_n295), .o1(\s[28] ));
  inv000aa1d42x5               g201(.a(\a[28] ), .o1(new_n297));
  xroi22aa1d04x5               g202(.a(\a[27] ), .b(new_n289), .c(new_n297), .d(\b[27] ), .out0(new_n298));
  nanp02aa1n03x5               g203(.a(new_n290), .b(new_n298), .o1(new_n299));
  inv000aa1d42x5               g204(.a(new_n298), .o1(new_n300));
  oaoi03aa1n12x5               g205(.a(\a[28] ), .b(\b[27] ), .c(new_n292), .o1(new_n301));
  inv000aa1d42x5               g206(.a(new_n301), .o1(new_n302));
  aoai13aa1n02x7               g207(.a(new_n302), .b(new_n300), .c(new_n280), .d(new_n276), .o1(new_n303));
  xorc02aa1n02x5               g208(.a(\a[29] ), .b(\b[28] ), .out0(new_n304));
  norp02aa1n02x5               g209(.a(new_n301), .b(new_n304), .o1(new_n305));
  aoi022aa1n03x5               g210(.a(new_n303), .b(new_n304), .c(new_n299), .d(new_n305), .o1(\s[29] ));
  xorb03aa1n02x5               g211(.a(new_n103), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  and003aa1n02x5               g212(.a(new_n294), .b(new_n304), .c(new_n283), .o(new_n308));
  nanp02aa1n03x5               g213(.a(new_n290), .b(new_n308), .o1(new_n309));
  inv000aa1d42x5               g214(.a(new_n308), .o1(new_n310));
  inv000aa1d42x5               g215(.a(\a[29] ), .o1(new_n311));
  inv000aa1d42x5               g216(.a(\b[28] ), .o1(new_n312));
  oaoi03aa1n02x5               g217(.a(new_n311), .b(new_n312), .c(new_n301), .o1(new_n313));
  aoai13aa1n02x7               g218(.a(new_n313), .b(new_n310), .c(new_n280), .d(new_n276), .o1(new_n314));
  xorc02aa1n02x5               g219(.a(\a[30] ), .b(\b[29] ), .out0(new_n315));
  oabi12aa1n02x5               g220(.a(new_n315), .b(\a[29] ), .c(\b[28] ), .out0(new_n316));
  oaoi13aa1n02x5               g221(.a(new_n316), .b(new_n301), .c(new_n311), .d(new_n312), .o1(new_n317));
  aoi022aa1n03x5               g222(.a(new_n314), .b(new_n315), .c(new_n309), .d(new_n317), .o1(\s[30] ));
  nano32aa1n02x4               g223(.a(new_n285), .b(new_n315), .c(new_n294), .d(new_n304), .out0(new_n319));
  nanp02aa1n03x5               g224(.a(new_n290), .b(new_n319), .o1(new_n320));
  xorc02aa1n02x5               g225(.a(\a[31] ), .b(\b[30] ), .out0(new_n321));
  oao003aa1n02x5               g226(.a(\a[30] ), .b(\b[29] ), .c(new_n313), .carry(new_n322));
  norb02aa1n02x5               g227(.a(new_n322), .b(new_n321), .out0(new_n323));
  inv000aa1n02x5               g228(.a(new_n319), .o1(new_n324));
  aoai13aa1n03x5               g229(.a(new_n322), .b(new_n324), .c(new_n280), .d(new_n276), .o1(new_n325));
  aoi022aa1n03x5               g230(.a(new_n325), .b(new_n321), .c(new_n320), .d(new_n323), .o1(\s[31] ));
  xobna2aa1n03x5               g231(.a(new_n109), .b(new_n104), .c(new_n102), .out0(\s[3] ));
  norp02aa1n02x5               g232(.a(\b[3] ), .b(\a[4] ), .o1(new_n328));
  aoi012aa1n02x5               g233(.a(new_n109), .b(new_n104), .c(new_n102), .o1(new_n329));
  xnrc02aa1n02x5               g234(.a(\b[3] ), .b(\a[4] ), .out0(new_n330));
  nano22aa1n02x4               g235(.a(new_n329), .b(new_n330), .c(new_n107), .out0(new_n331));
  inv000aa1d42x5               g236(.a(\b[3] ), .o1(new_n332));
  oaib12aa1n02x5               g237(.a(new_n112), .b(new_n332), .c(\a[4] ), .out0(new_n333));
  oab012aa1n02x4               g238(.a(new_n331), .b(new_n333), .c(new_n328), .out0(\s[4] ));
  xnrb03aa1n02x5               g239(.a(new_n333), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  oaoi03aa1n02x5               g240(.a(\a[5] ), .b(\b[4] ), .c(new_n333), .o1(new_n336));
  xorb03aa1n02x5               g241(.a(new_n336), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  norb02aa1n02x5               g242(.a(new_n121), .b(new_n120), .out0(new_n338));
  oai112aa1n02x5               g243(.a(new_n126), .b(new_n338), .c(new_n336), .d(new_n122), .o1(new_n339));
  oaoi13aa1n02x5               g244(.a(new_n338), .b(new_n126), .c(new_n336), .d(new_n122), .o1(new_n340));
  norb02aa1n02x5               g245(.a(new_n339), .b(new_n340), .out0(\s[7] ));
  orn002aa1n02x5               g246(.a(\a[7] ), .b(\b[6] ), .o(new_n342));
  xnbna2aa1n03x5               g247(.a(new_n129), .b(new_n339), .c(new_n342), .out0(\s[8] ));
  xorb03aa1n02x5               g248(.a(new_n132), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


