// Benchmark "adder" written by ABC on Wed Jul 17 19:07:15 2024

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
    new_n125, new_n126, new_n127, new_n128, new_n130, new_n131, new_n132,
    new_n133, new_n134, new_n135, new_n136, new_n138, new_n139, new_n140,
    new_n141, new_n142, new_n143, new_n144, new_n145, new_n146, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n155, new_n156,
    new_n157, new_n158, new_n159, new_n160, new_n161, new_n162, new_n164,
    new_n165, new_n166, new_n167, new_n169, new_n170, new_n171, new_n172,
    new_n173, new_n174, new_n175, new_n176, new_n177, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n186, new_n187,
    new_n188, new_n190, new_n191, new_n192, new_n193, new_n194, new_n195,
    new_n196, new_n198, new_n199, new_n200, new_n201, new_n202, new_n203,
    new_n204, new_n207, new_n208, new_n209, new_n210, new_n211, new_n212,
    new_n213, new_n214, new_n215, new_n216, new_n218, new_n219, new_n220,
    new_n221, new_n222, new_n223, new_n224, new_n225, new_n227, new_n228,
    new_n229, new_n230, new_n231, new_n232, new_n233, new_n234, new_n235,
    new_n236, new_n238, new_n239, new_n240, new_n241, new_n242, new_n244,
    new_n245, new_n246, new_n247, new_n248, new_n249, new_n250, new_n251,
    new_n252, new_n253, new_n254, new_n255, new_n257, new_n258, new_n259,
    new_n260, new_n261, new_n262, new_n263, new_n264, new_n266, new_n267,
    new_n268, new_n269, new_n270, new_n271, new_n272, new_n273, new_n274,
    new_n276, new_n277, new_n278, new_n279, new_n280, new_n281, new_n282,
    new_n283, new_n284, new_n286, new_n287, new_n288, new_n289, new_n290,
    new_n291, new_n292, new_n293, new_n295, new_n296, new_n297, new_n298,
    new_n299, new_n300, new_n301, new_n302, new_n303, new_n306, new_n307,
    new_n308, new_n309, new_n310, new_n311, new_n312, new_n314, new_n315,
    new_n316, new_n317, new_n318, new_n319, new_n320, new_n323, new_n326,
    new_n328, new_n329, new_n331, new_n332, new_n333, new_n334, new_n335;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor002aa1d32x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  inv000aa1d42x5               g002(.a(new_n97), .o1(new_n98));
  nor042aa1n02x5               g003(.a(\b[1] ), .b(\a[2] ), .o1(new_n99));
  nanp02aa1n04x5               g004(.a(\b[0] ), .b(\a[1] ), .o1(new_n100));
  nanp02aa1n02x5               g005(.a(\b[1] ), .b(\a[2] ), .o1(new_n101));
  aoi012aa1n12x5               g006(.a(new_n99), .b(new_n100), .c(new_n101), .o1(new_n102));
  tech160nm_fixnrc02aa1n04x5   g007(.a(\b[3] ), .b(\a[4] ), .out0(new_n103));
  nor042aa1n06x5               g008(.a(\b[2] ), .b(\a[3] ), .o1(new_n104));
  nand42aa1n03x5               g009(.a(\b[2] ), .b(\a[3] ), .o1(new_n105));
  nanb02aa1n06x5               g010(.a(new_n104), .b(new_n105), .out0(new_n106));
  inv000aa1n06x5               g011(.a(new_n104), .o1(new_n107));
  tech160nm_fioaoi03aa1n03p5x5 g012(.a(\a[4] ), .b(\b[3] ), .c(new_n107), .o1(new_n108));
  inv040aa1n02x5               g013(.a(new_n108), .o1(new_n109));
  oai013aa1n09x5               g014(.a(new_n109), .b(new_n103), .c(new_n102), .d(new_n106), .o1(new_n110));
  nor022aa1n08x5               g015(.a(\b[7] ), .b(\a[8] ), .o1(new_n111));
  nand02aa1d06x5               g016(.a(\b[7] ), .b(\a[8] ), .o1(new_n112));
  nor022aa1n08x5               g017(.a(\b[6] ), .b(\a[7] ), .o1(new_n113));
  nand42aa1n03x5               g018(.a(\b[6] ), .b(\a[7] ), .o1(new_n114));
  nona23aa1n09x5               g019(.a(new_n114), .b(new_n112), .c(new_n111), .d(new_n113), .out0(new_n115));
  xorc02aa1n02x5               g020(.a(\a[5] ), .b(\b[4] ), .out0(new_n116));
  tech160nm_fixnrc02aa1n04x5   g021(.a(\b[5] ), .b(\a[6] ), .out0(new_n117));
  norb03aa1n06x5               g022(.a(new_n116), .b(new_n115), .c(new_n117), .out0(new_n118));
  nor042aa1n03x5               g023(.a(\b[4] ), .b(\a[5] ), .o1(new_n119));
  inv000aa1n02x5               g024(.a(new_n119), .o1(new_n120));
  oaoi03aa1n02x5               g025(.a(\a[6] ), .b(\b[5] ), .c(new_n120), .o1(new_n121));
  aoi012aa1n02x5               g026(.a(new_n111), .b(new_n113), .c(new_n112), .o1(new_n122));
  oaib12aa1n02x7               g027(.a(new_n122), .b(new_n115), .c(new_n121), .out0(new_n123));
  tech160nm_fixorc02aa1n02p5x5 g028(.a(\a[9] ), .b(\b[8] ), .out0(new_n124));
  aoai13aa1n02x5               g029(.a(new_n124), .b(new_n123), .c(new_n118), .d(new_n110), .o1(new_n125));
  norp02aa1n02x5               g030(.a(\b[9] ), .b(\a[10] ), .o1(new_n126));
  nand42aa1n03x5               g031(.a(\b[9] ), .b(\a[10] ), .o1(new_n127));
  norb02aa1n03x4               g032(.a(new_n127), .b(new_n126), .out0(new_n128));
  xnbna2aa1n03x5               g033(.a(new_n128), .b(new_n125), .c(new_n98), .out0(\s[10] ));
  norp03aa1n04x5               g034(.a(new_n103), .b(new_n102), .c(new_n106), .o1(new_n130));
  tech160nm_fioai012aa1n05x5   g035(.a(new_n118), .b(new_n130), .c(new_n108), .o1(new_n131));
  tech160nm_fiao0012aa1n02p5x5 g036(.a(new_n111), .b(new_n113), .c(new_n112), .o(new_n132));
  aoib12aa1n03x5               g037(.a(new_n132), .b(new_n121), .c(new_n115), .out0(new_n133));
  nanp02aa1n02x5               g038(.a(new_n124), .b(new_n128), .o1(new_n134));
  oai012aa1n02x7               g039(.a(new_n127), .b(new_n126), .c(new_n97), .o1(new_n135));
  aoai13aa1n02x5               g040(.a(new_n135), .b(new_n134), .c(new_n131), .d(new_n133), .o1(new_n136));
  xorb03aa1n02x5               g041(.a(new_n136), .b(\b[10] ), .c(\a[11] ), .out0(\s[11] ));
  nor042aa1n04x5               g042(.a(\b[10] ), .b(\a[11] ), .o1(new_n138));
  nand22aa1n03x5               g043(.a(\b[10] ), .b(\a[11] ), .o1(new_n139));
  aoi012aa1n02x5               g044(.a(new_n138), .b(new_n136), .c(new_n139), .o1(new_n140));
  nor002aa1n03x5               g045(.a(\b[11] ), .b(\a[12] ), .o1(new_n141));
  nand42aa1n02x5               g046(.a(\b[11] ), .b(\a[12] ), .o1(new_n142));
  norb02aa1n02x5               g047(.a(new_n142), .b(new_n141), .out0(new_n143));
  norb02aa1n02x5               g048(.a(new_n139), .b(new_n138), .out0(new_n144));
  norb03aa1n02x5               g049(.a(new_n142), .b(new_n138), .c(new_n141), .out0(new_n145));
  aob012aa1n02x5               g050(.a(new_n145), .b(new_n136), .c(new_n144), .out0(new_n146));
  oai012aa1n02x5               g051(.a(new_n146), .b(new_n140), .c(new_n143), .o1(\s[12] ));
  nona23aa1n03x5               g052(.a(new_n142), .b(new_n139), .c(new_n138), .d(new_n141), .out0(new_n148));
  nano22aa1n02x5               g053(.a(new_n148), .b(new_n124), .c(new_n128), .out0(new_n149));
  aoai13aa1n06x5               g054(.a(new_n149), .b(new_n123), .c(new_n118), .d(new_n110), .o1(new_n150));
  oai012aa1n02x5               g055(.a(new_n142), .b(new_n141), .c(new_n138), .o1(new_n151));
  tech160nm_fioai012aa1n05x5   g056(.a(new_n151), .b(new_n148), .c(new_n135), .o1(new_n152));
  nanb02aa1n02x5               g057(.a(new_n152), .b(new_n150), .out0(new_n153));
  xorb03aa1n02x5               g058(.a(new_n153), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  nor042aa1n04x5               g059(.a(\b[12] ), .b(\a[13] ), .o1(new_n155));
  nanp02aa1n03x5               g060(.a(\b[12] ), .b(\a[13] ), .o1(new_n156));
  nano22aa1n03x7               g061(.a(new_n155), .b(new_n153), .c(new_n156), .out0(new_n157));
  nor042aa1n04x5               g062(.a(\b[13] ), .b(\a[14] ), .o1(new_n158));
  nanp02aa1n04x5               g063(.a(\b[13] ), .b(\a[14] ), .o1(new_n159));
  nanb02aa1n02x5               g064(.a(new_n158), .b(new_n159), .out0(new_n160));
  aoai13aa1n02x5               g065(.a(new_n160), .b(new_n155), .c(new_n153), .d(new_n156), .o1(new_n161));
  nona22aa1n02x4               g066(.a(new_n159), .b(new_n158), .c(new_n155), .out0(new_n162));
  oai012aa1n02x5               g067(.a(new_n161), .b(new_n157), .c(new_n162), .o1(\s[14] ));
  nona23aa1n09x5               g068(.a(new_n159), .b(new_n156), .c(new_n155), .d(new_n158), .out0(new_n164));
  nano23aa1n06x5               g069(.a(new_n155), .b(new_n158), .c(new_n159), .d(new_n156), .out0(new_n165));
  aoi022aa1n02x5               g070(.a(new_n152), .b(new_n165), .c(new_n159), .d(new_n162), .o1(new_n166));
  tech160nm_fioai012aa1n05x5   g071(.a(new_n166), .b(new_n150), .c(new_n164), .o1(new_n167));
  xorb03aa1n02x5               g072(.a(new_n167), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  nor042aa1n03x5               g073(.a(\b[14] ), .b(\a[15] ), .o1(new_n169));
  nanp02aa1n02x5               g074(.a(\b[14] ), .b(\a[15] ), .o1(new_n170));
  nanb02aa1n02x5               g075(.a(new_n169), .b(new_n170), .out0(new_n171));
  oaoi13aa1n02x5               g076(.a(new_n171), .b(new_n166), .c(new_n150), .d(new_n164), .o1(new_n172));
  aoi012aa1n02x5               g077(.a(new_n169), .b(new_n167), .c(new_n170), .o1(new_n173));
  nor042aa1n02x5               g078(.a(\b[15] ), .b(\a[16] ), .o1(new_n174));
  nand42aa1n02x5               g079(.a(\b[15] ), .b(\a[16] ), .o1(new_n175));
  nanb02aa1n03x5               g080(.a(new_n174), .b(new_n175), .out0(new_n176));
  nona22aa1n02x4               g081(.a(new_n175), .b(new_n174), .c(new_n169), .out0(new_n177));
  obai22aa1n02x7               g082(.a(new_n176), .b(new_n173), .c(new_n172), .d(new_n177), .out0(\s[16] ));
  nano23aa1n02x5               g083(.a(new_n138), .b(new_n141), .c(new_n142), .d(new_n139), .out0(new_n179));
  nano23aa1n02x4               g084(.a(new_n169), .b(new_n174), .c(new_n175), .d(new_n170), .out0(new_n180));
  nano32aa1n03x7               g085(.a(new_n134), .b(new_n180), .c(new_n179), .d(new_n165), .out0(new_n181));
  aoai13aa1n06x5               g086(.a(new_n181), .b(new_n123), .c(new_n110), .d(new_n118), .o1(new_n182));
  nor003aa1n03x5               g087(.a(new_n164), .b(new_n171), .c(new_n176), .o1(new_n183));
  oai012aa1n02x5               g088(.a(new_n159), .b(new_n158), .c(new_n155), .o1(new_n184));
  aoi012aa1n02x5               g089(.a(new_n174), .b(new_n169), .c(new_n175), .o1(new_n185));
  oai013aa1n03x4               g090(.a(new_n185), .b(new_n184), .c(new_n171), .d(new_n176), .o1(new_n186));
  aoi012aa1n09x5               g091(.a(new_n186), .b(new_n152), .c(new_n183), .o1(new_n187));
  xorc02aa1n02x5               g092(.a(\a[17] ), .b(\b[16] ), .out0(new_n188));
  xnbna2aa1n03x5               g093(.a(new_n188), .b(new_n182), .c(new_n187), .out0(\s[17] ));
  nand02aa1n02x5               g094(.a(new_n149), .b(new_n183), .o1(new_n190));
  aoai13aa1n09x5               g095(.a(new_n187), .b(new_n190), .c(new_n131), .d(new_n133), .o1(new_n191));
  nor002aa1d32x5               g096(.a(\b[16] ), .b(\a[17] ), .o1(new_n192));
  xnrc02aa1n02x5               g097(.a(\b[17] ), .b(\a[18] ), .out0(new_n193));
  aoai13aa1n02x5               g098(.a(new_n193), .b(new_n192), .c(new_n191), .d(new_n188), .o1(new_n194));
  inv040aa1n08x5               g099(.a(new_n192), .o1(new_n195));
  nanb02aa1n02x5               g100(.a(new_n193), .b(new_n195), .out0(new_n196));
  aoai13aa1n02x5               g101(.a(new_n194), .b(new_n196), .c(new_n188), .d(new_n191), .o1(\s[18] ));
  inv000aa1d42x5               g102(.a(\a[17] ), .o1(new_n198));
  inv020aa1n04x5               g103(.a(\a[18] ), .o1(new_n199));
  xroi22aa1d06x4               g104(.a(new_n198), .b(\b[16] ), .c(new_n199), .d(\b[17] ), .out0(new_n200));
  inv000aa1d42x5               g105(.a(new_n200), .o1(new_n201));
  oaoi03aa1n12x5               g106(.a(\a[18] ), .b(\b[17] ), .c(new_n195), .o1(new_n202));
  inv000aa1d42x5               g107(.a(new_n202), .o1(new_n203));
  aoai13aa1n03x5               g108(.a(new_n203), .b(new_n201), .c(new_n182), .d(new_n187), .o1(new_n204));
  xorb03aa1n02x5               g109(.a(new_n204), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g110(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor042aa1n09x5               g111(.a(\b[18] ), .b(\a[19] ), .o1(new_n207));
  nand42aa1n20x5               g112(.a(\b[18] ), .b(\a[19] ), .o1(new_n208));
  nor042aa1n09x5               g113(.a(\b[19] ), .b(\a[20] ), .o1(new_n209));
  nand42aa1n20x5               g114(.a(\b[19] ), .b(\a[20] ), .o1(new_n210));
  nanb02aa1n02x5               g115(.a(new_n209), .b(new_n210), .out0(new_n211));
  aoai13aa1n03x5               g116(.a(new_n211), .b(new_n207), .c(new_n204), .d(new_n208), .o1(new_n212));
  nanp02aa1n03x5               g117(.a(new_n191), .b(new_n200), .o1(new_n213));
  nanb02aa1n02x5               g118(.a(new_n207), .b(new_n208), .out0(new_n214));
  norb03aa1n02x5               g119(.a(new_n210), .b(new_n207), .c(new_n209), .out0(new_n215));
  aoai13aa1n03x5               g120(.a(new_n215), .b(new_n214), .c(new_n213), .d(new_n203), .o1(new_n216));
  nanp02aa1n03x5               g121(.a(new_n212), .b(new_n216), .o1(\s[20] ));
  nano23aa1d15x5               g122(.a(new_n207), .b(new_n209), .c(new_n210), .d(new_n208), .out0(new_n218));
  nano22aa1n03x7               g123(.a(new_n193), .b(new_n218), .c(new_n188), .out0(new_n219));
  inv000aa1d42x5               g124(.a(new_n219), .o1(new_n220));
  nand02aa1d06x5               g125(.a(new_n218), .b(new_n202), .o1(new_n221));
  oaih12aa1n12x5               g126(.a(new_n210), .b(new_n209), .c(new_n207), .o1(new_n222));
  nand02aa1n04x5               g127(.a(new_n221), .b(new_n222), .o1(new_n223));
  inv000aa1d42x5               g128(.a(new_n223), .o1(new_n224));
  aoai13aa1n03x5               g129(.a(new_n224), .b(new_n220), .c(new_n182), .d(new_n187), .o1(new_n225));
  xorb03aa1n02x5               g130(.a(new_n225), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor042aa1n06x5               g131(.a(\b[20] ), .b(\a[21] ), .o1(new_n227));
  nand22aa1n12x5               g132(.a(\b[20] ), .b(\a[21] ), .o1(new_n228));
  norb02aa1n02x5               g133(.a(new_n228), .b(new_n227), .out0(new_n229));
  nor002aa1n03x5               g134(.a(\b[21] ), .b(\a[22] ), .o1(new_n230));
  nand22aa1n12x5               g135(.a(\b[21] ), .b(\a[22] ), .o1(new_n231));
  norb02aa1n02x5               g136(.a(new_n231), .b(new_n230), .out0(new_n232));
  inv000aa1d42x5               g137(.a(new_n232), .o1(new_n233));
  aoai13aa1n03x5               g138(.a(new_n233), .b(new_n227), .c(new_n225), .d(new_n229), .o1(new_n234));
  aoai13aa1n03x5               g139(.a(new_n229), .b(new_n223), .c(new_n191), .d(new_n219), .o1(new_n235));
  nona22aa1n03x5               g140(.a(new_n235), .b(new_n233), .c(new_n227), .out0(new_n236));
  nanp02aa1n03x5               g141(.a(new_n234), .b(new_n236), .o1(\s[22] ));
  nano23aa1n06x5               g142(.a(new_n227), .b(new_n230), .c(new_n231), .d(new_n228), .out0(new_n238));
  nanp03aa1n02x5               g143(.a(new_n200), .b(new_n218), .c(new_n238), .o1(new_n239));
  ao0012aa1n03x7               g144(.a(new_n230), .b(new_n227), .c(new_n231), .o(new_n240));
  tech160nm_fiaoi012aa1n05x5   g145(.a(new_n240), .b(new_n223), .c(new_n238), .o1(new_n241));
  aoai13aa1n06x5               g146(.a(new_n241), .b(new_n239), .c(new_n182), .d(new_n187), .o1(new_n242));
  xorb03aa1n02x5               g147(.a(new_n242), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  nor002aa1d32x5               g148(.a(\b[22] ), .b(\a[23] ), .o1(new_n244));
  nand42aa1n04x5               g149(.a(\b[22] ), .b(\a[23] ), .o1(new_n245));
  norb02aa1n02x5               g150(.a(new_n245), .b(new_n244), .out0(new_n246));
  nor002aa1d32x5               g151(.a(\b[23] ), .b(\a[24] ), .o1(new_n247));
  nand42aa1d28x5               g152(.a(\b[23] ), .b(\a[24] ), .o1(new_n248));
  norb02aa1n02x5               g153(.a(new_n248), .b(new_n247), .out0(new_n249));
  inv000aa1d42x5               g154(.a(new_n249), .o1(new_n250));
  aoai13aa1n03x5               g155(.a(new_n250), .b(new_n244), .c(new_n242), .d(new_n246), .o1(new_n251));
  nanp02aa1n02x5               g156(.a(new_n242), .b(new_n246), .o1(new_n252));
  nona22aa1d36x5               g157(.a(new_n248), .b(new_n247), .c(new_n244), .out0(new_n253));
  inv000aa1d42x5               g158(.a(new_n253), .o1(new_n254));
  nanp02aa1n02x5               g159(.a(new_n252), .b(new_n254), .o1(new_n255));
  nanp02aa1n02x5               g160(.a(new_n251), .b(new_n255), .o1(\s[24] ));
  nano23aa1n09x5               g161(.a(new_n244), .b(new_n247), .c(new_n248), .d(new_n245), .out0(new_n257));
  nand22aa1n09x5               g162(.a(new_n257), .b(new_n238), .o1(new_n258));
  inv000aa1d42x5               g163(.a(new_n258), .o1(new_n259));
  nanp02aa1n02x5               g164(.a(new_n219), .b(new_n259), .o1(new_n260));
  aoi022aa1n12x5               g165(.a(new_n257), .b(new_n240), .c(new_n248), .d(new_n253), .o1(new_n261));
  aoai13aa1n12x5               g166(.a(new_n261), .b(new_n258), .c(new_n221), .d(new_n222), .o1(new_n262));
  inv000aa1d42x5               g167(.a(new_n262), .o1(new_n263));
  aoai13aa1n04x5               g168(.a(new_n263), .b(new_n260), .c(new_n182), .d(new_n187), .o1(new_n264));
  xorb03aa1n02x5               g169(.a(new_n264), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g170(.a(\b[24] ), .b(\a[25] ), .o1(new_n266));
  xnrc02aa1n12x5               g171(.a(\b[24] ), .b(\a[25] ), .out0(new_n267));
  inv000aa1d42x5               g172(.a(new_n267), .o1(new_n268));
  xnrc02aa1n02x5               g173(.a(\b[25] ), .b(\a[26] ), .out0(new_n269));
  aoai13aa1n02x5               g174(.a(new_n269), .b(new_n266), .c(new_n264), .d(new_n268), .o1(new_n270));
  inv000aa1d42x5               g175(.a(new_n218), .o1(new_n271));
  nona32aa1n06x5               g176(.a(new_n191), .b(new_n258), .c(new_n271), .d(new_n201), .out0(new_n272));
  norp02aa1n02x5               g177(.a(new_n269), .b(new_n266), .o1(new_n273));
  aoai13aa1n03x5               g178(.a(new_n273), .b(new_n267), .c(new_n272), .d(new_n263), .o1(new_n274));
  nanp02aa1n02x5               g179(.a(new_n270), .b(new_n274), .o1(\s[26] ));
  norp02aa1n04x5               g180(.a(new_n269), .b(new_n267), .o1(new_n276));
  nano32aa1n03x7               g181(.a(new_n258), .b(new_n200), .c(new_n276), .d(new_n218), .out0(new_n277));
  inv020aa1n03x5               g182(.a(new_n277), .o1(new_n278));
  inv000aa1d42x5               g183(.a(\a[26] ), .o1(new_n279));
  inv000aa1d42x5               g184(.a(\b[25] ), .o1(new_n280));
  oaoi03aa1n02x5               g185(.a(new_n279), .b(new_n280), .c(new_n266), .o1(new_n281));
  inv000aa1n02x5               g186(.a(new_n281), .o1(new_n282));
  tech160nm_fiaoi012aa1n05x5   g187(.a(new_n282), .b(new_n262), .c(new_n276), .o1(new_n283));
  aoai13aa1n06x5               g188(.a(new_n283), .b(new_n278), .c(new_n182), .d(new_n187), .o1(new_n284));
  xorb03aa1n03x5               g189(.a(new_n284), .b(\b[26] ), .c(\a[27] ), .out0(\s[27] ));
  nor042aa1n03x5               g190(.a(\b[26] ), .b(\a[27] ), .o1(new_n286));
  xorc02aa1n02x5               g191(.a(\a[27] ), .b(\b[26] ), .out0(new_n287));
  xnrc02aa1n02x5               g192(.a(\b[27] ), .b(\a[28] ), .out0(new_n288));
  aoai13aa1n03x5               g193(.a(new_n288), .b(new_n286), .c(new_n284), .d(new_n287), .o1(new_n289));
  nanp02aa1n06x5               g194(.a(new_n262), .b(new_n276), .o1(new_n290));
  nanp02aa1n06x5               g195(.a(new_n290), .b(new_n281), .o1(new_n291));
  aoai13aa1n02x7               g196(.a(new_n287), .b(new_n291), .c(new_n191), .d(new_n277), .o1(new_n292));
  nona22aa1n03x5               g197(.a(new_n292), .b(new_n288), .c(new_n286), .out0(new_n293));
  nanp02aa1n03x5               g198(.a(new_n289), .b(new_n293), .o1(\s[28] ));
  norb02aa1n02x5               g199(.a(new_n287), .b(new_n288), .out0(new_n295));
  aoai13aa1n02x7               g200(.a(new_n295), .b(new_n291), .c(new_n191), .d(new_n277), .o1(new_n296));
  xnrc02aa1n02x5               g201(.a(\b[28] ), .b(\a[29] ), .out0(new_n297));
  inv000aa1d42x5               g202(.a(\a[28] ), .o1(new_n298));
  inv000aa1d42x5               g203(.a(\b[27] ), .o1(new_n299));
  oaoi03aa1n09x5               g204(.a(new_n298), .b(new_n299), .c(new_n286), .o1(new_n300));
  inv000aa1d42x5               g205(.a(new_n300), .o1(new_n301));
  nona22aa1n03x5               g206(.a(new_n296), .b(new_n297), .c(new_n301), .out0(new_n302));
  aoai13aa1n03x5               g207(.a(new_n297), .b(new_n301), .c(new_n284), .d(new_n295), .o1(new_n303));
  nanp02aa1n03x5               g208(.a(new_n303), .b(new_n302), .o1(\s[29] ));
  xorb03aa1n02x5               g209(.a(new_n100), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g210(.a(new_n287), .b(new_n297), .c(new_n288), .out0(new_n306));
  oaoi03aa1n02x5               g211(.a(\a[29] ), .b(\b[28] ), .c(new_n300), .o1(new_n307));
  tech160nm_fixorc02aa1n03p5x5 g212(.a(\a[30] ), .b(\b[29] ), .out0(new_n308));
  inv000aa1d42x5               g213(.a(new_n308), .o1(new_n309));
  aoai13aa1n03x5               g214(.a(new_n309), .b(new_n307), .c(new_n284), .d(new_n306), .o1(new_n310));
  aoai13aa1n02x7               g215(.a(new_n306), .b(new_n291), .c(new_n191), .d(new_n277), .o1(new_n311));
  nona22aa1n03x5               g216(.a(new_n311), .b(new_n307), .c(new_n309), .out0(new_n312));
  nanp02aa1n03x5               g217(.a(new_n310), .b(new_n312), .o1(\s[30] ));
  xnrc02aa1n02x5               g218(.a(\b[30] ), .b(\a[31] ), .out0(new_n314));
  nano23aa1n02x4               g219(.a(new_n297), .b(new_n288), .c(new_n308), .d(new_n287), .out0(new_n315));
  and002aa1n02x5               g220(.a(\b[29] ), .b(\a[30] ), .o(new_n316));
  oab012aa1n02x4               g221(.a(new_n316), .b(new_n307), .c(new_n309), .out0(new_n317));
  aoai13aa1n03x5               g222(.a(new_n314), .b(new_n317), .c(new_n284), .d(new_n315), .o1(new_n318));
  aoai13aa1n02x7               g223(.a(new_n315), .b(new_n291), .c(new_n191), .d(new_n277), .o1(new_n319));
  nona22aa1n03x5               g224(.a(new_n319), .b(new_n317), .c(new_n314), .out0(new_n320));
  nanp02aa1n03x5               g225(.a(new_n318), .b(new_n320), .o1(\s[31] ));
  xnbna2aa1n03x5               g226(.a(new_n102), .b(new_n105), .c(new_n107), .out0(\s[3] ));
  orn002aa1n02x5               g227(.a(new_n102), .b(new_n106), .o(new_n323));
  xobna2aa1n03x5               g228(.a(new_n103), .b(new_n323), .c(new_n107), .out0(\s[4] ));
  xorb03aa1n02x5               g229(.a(new_n110), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  oai012aa1n02x5               g230(.a(new_n116), .b(new_n130), .c(new_n108), .o1(new_n326));
  xobna2aa1n03x5               g231(.a(new_n117), .b(new_n326), .c(new_n120), .out0(\s[6] ));
  nona22aa1n02x4               g232(.a(new_n326), .b(new_n117), .c(new_n119), .out0(new_n328));
  aob012aa1n02x5               g233(.a(new_n328), .b(\b[5] ), .c(\a[6] ), .out0(new_n329));
  xnrb03aa1n02x5               g234(.a(new_n329), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  nanb02aa1n02x5               g235(.a(new_n111), .b(new_n112), .out0(new_n331));
  oaoi03aa1n02x5               g236(.a(\a[7] ), .b(\b[6] ), .c(new_n329), .o1(new_n332));
  nanb02aa1n02x5               g237(.a(new_n113), .b(new_n114), .out0(new_n333));
  norb03aa1n02x5               g238(.a(new_n112), .b(new_n111), .c(new_n113), .out0(new_n334));
  oai012aa1n02x5               g239(.a(new_n334), .b(new_n329), .c(new_n333), .o1(new_n335));
  aob012aa1n02x5               g240(.a(new_n335), .b(new_n332), .c(new_n331), .out0(\s[8] ));
  xnbna2aa1n03x5               g241(.a(new_n124), .b(new_n131), .c(new_n133), .out0(\s[9] ));
endmodule


