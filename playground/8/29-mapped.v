// Benchmark "adder" written by ABC on Wed Jul 17 16:17:32 2024

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
    new_n133, new_n134, new_n135, new_n136, new_n137, new_n138, new_n139,
    new_n140, new_n141, new_n142, new_n143, new_n144, new_n146, new_n147,
    new_n148, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n159, new_n160, new_n161, new_n162,
    new_n164, new_n165, new_n166, new_n168, new_n169, new_n170, new_n171,
    new_n172, new_n173, new_n174, new_n176, new_n177, new_n178, new_n179,
    new_n180, new_n181, new_n182, new_n183, new_n184, new_n185, new_n187,
    new_n188, new_n189, new_n190, new_n191, new_n192, new_n193, new_n194,
    new_n195, new_n196, new_n197, new_n199, new_n200, new_n201, new_n202,
    new_n203, new_n205, new_n206, new_n207, new_n208, new_n209, new_n210,
    new_n213, new_n214, new_n215, new_n216, new_n217, new_n218, new_n219,
    new_n220, new_n221, new_n222, new_n223, new_n224, new_n225, new_n227,
    new_n228, new_n229, new_n230, new_n231, new_n232, new_n233, new_n235,
    new_n236, new_n237, new_n238, new_n239, new_n240, new_n241, new_n242,
    new_n243, new_n244, new_n245, new_n247, new_n248, new_n249, new_n250,
    new_n251, new_n252, new_n253, new_n254, new_n255, new_n256, new_n257,
    new_n258, new_n259, new_n261, new_n262, new_n263, new_n264, new_n265,
    new_n266, new_n267, new_n269, new_n270, new_n271, new_n272, new_n273,
    new_n274, new_n275, new_n276, new_n277, new_n278, new_n279, new_n280,
    new_n281, new_n282, new_n284, new_n285, new_n286, new_n287, new_n288,
    new_n290, new_n291, new_n292, new_n293, new_n294, new_n295, new_n296,
    new_n298, new_n299, new_n300, new_n301, new_n302, new_n303, new_n304,
    new_n305, new_n306, new_n307, new_n308, new_n309, new_n311, new_n312,
    new_n313, new_n314, new_n315, new_n316, new_n317, new_n318, new_n319,
    new_n320, new_n323, new_n324, new_n325, new_n326, new_n327, new_n328,
    new_n329, new_n331, new_n332, new_n333, new_n334, new_n335, new_n336,
    new_n339, new_n342, new_n344, new_n345, new_n346, new_n347, new_n349;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  inv000aa1d42x5               g001(.a(\a[10] ), .o1(new_n97));
  nor042aa1n03x5               g002(.a(\b[8] ), .b(\a[9] ), .o1(new_n98));
  and002aa1n02x5               g003(.a(\b[0] ), .b(\a[1] ), .o(new_n99));
  oaoi03aa1n02x5               g004(.a(\a[2] ), .b(\b[1] ), .c(new_n99), .o1(new_n100));
  norp02aa1n06x5               g005(.a(\b[3] ), .b(\a[4] ), .o1(new_n101));
  nand22aa1n12x5               g006(.a(\b[3] ), .b(\a[4] ), .o1(new_n102));
  nor022aa1n16x5               g007(.a(\b[2] ), .b(\a[3] ), .o1(new_n103));
  tech160nm_finand02aa1n03p5x5 g008(.a(\b[2] ), .b(\a[3] ), .o1(new_n104));
  nano23aa1n02x5               g009(.a(new_n101), .b(new_n103), .c(new_n104), .d(new_n102), .out0(new_n105));
  inv040aa1n02x5               g010(.a(new_n101), .o1(new_n106));
  aob012aa1n06x5               g011(.a(new_n106), .b(new_n103), .c(new_n102), .out0(new_n107));
  tech160nm_fiaoi012aa1n04x5   g012(.a(new_n107), .b(new_n105), .c(new_n100), .o1(new_n108));
  nor042aa1n04x5               g013(.a(\b[4] ), .b(\a[5] ), .o1(new_n109));
  nanp02aa1n04x5               g014(.a(\b[4] ), .b(\a[5] ), .o1(new_n110));
  nanb02aa1n09x5               g015(.a(new_n109), .b(new_n110), .out0(new_n111));
  nor042aa1n03x5               g016(.a(\b[5] ), .b(\a[6] ), .o1(new_n112));
  nand02aa1d28x5               g017(.a(\b[5] ), .b(\a[6] ), .o1(new_n113));
  nanb02aa1n12x5               g018(.a(new_n112), .b(new_n113), .out0(new_n114));
  nor002aa1n03x5               g019(.a(\b[6] ), .b(\a[7] ), .o1(new_n115));
  nanp02aa1n04x5               g020(.a(\b[6] ), .b(\a[7] ), .o1(new_n116));
  nor002aa1n16x5               g021(.a(\b[7] ), .b(\a[8] ), .o1(new_n117));
  nand22aa1n12x5               g022(.a(\b[7] ), .b(\a[8] ), .o1(new_n118));
  nano23aa1n03x7               g023(.a(new_n115), .b(new_n117), .c(new_n118), .d(new_n116), .out0(new_n119));
  nona22aa1n03x5               g024(.a(new_n119), .b(new_n114), .c(new_n111), .out0(new_n120));
  inv000aa1d42x5               g025(.a(\b[6] ), .o1(new_n121));
  nanb02aa1n02x5               g026(.a(\a[7] ), .b(new_n121), .out0(new_n122));
  aoai13aa1n06x5               g027(.a(new_n116), .b(new_n112), .c(new_n109), .d(new_n113), .o1(new_n123));
  nand22aa1n03x5               g028(.a(new_n123), .b(new_n122), .o1(new_n124));
  aoi012aa1n12x5               g029(.a(new_n117), .b(new_n124), .c(new_n118), .o1(new_n125));
  oai012aa1n06x5               g030(.a(new_n125), .b(new_n108), .c(new_n120), .o1(new_n126));
  xorc02aa1n12x5               g031(.a(\a[9] ), .b(\b[8] ), .out0(new_n127));
  aoi012aa1n02x5               g032(.a(new_n98), .b(new_n126), .c(new_n127), .o1(new_n128));
  xorb03aa1n02x5               g033(.a(new_n128), .b(\b[9] ), .c(new_n97), .out0(\s[10] ));
  inv040aa1d32x5               g034(.a(\a[1] ), .o1(new_n130));
  inv040aa1d32x5               g035(.a(\b[0] ), .o1(new_n131));
  nor022aa1n08x5               g036(.a(\b[1] ), .b(\a[2] ), .o1(new_n132));
  nand02aa1n03x5               g037(.a(\b[1] ), .b(\a[2] ), .o1(new_n133));
  oaoi13aa1n12x5               g038(.a(new_n132), .b(new_n133), .c(new_n130), .d(new_n131), .o1(new_n134));
  nona23aa1n09x5               g039(.a(new_n104), .b(new_n102), .c(new_n101), .d(new_n103), .out0(new_n135));
  oabi12aa1n18x5               g040(.a(new_n107), .b(new_n135), .c(new_n134), .out0(new_n136));
  nona23aa1n09x5               g041(.a(new_n118), .b(new_n116), .c(new_n115), .d(new_n117), .out0(new_n137));
  nor043aa1n09x5               g042(.a(new_n137), .b(new_n114), .c(new_n111), .o1(new_n138));
  nanp02aa1n09x5               g043(.a(new_n136), .b(new_n138), .o1(new_n139));
  tech160nm_fixorc02aa1n05x5   g044(.a(\a[10] ), .b(\b[9] ), .out0(new_n140));
  nanp02aa1n02x5               g045(.a(new_n127), .b(new_n140), .o1(new_n141));
  inv000aa1d42x5               g046(.a(\b[9] ), .o1(new_n142));
  tech160nm_fioaoi03aa1n03p5x5 g047(.a(new_n97), .b(new_n142), .c(new_n98), .o1(new_n143));
  aoai13aa1n03x5               g048(.a(new_n143), .b(new_n141), .c(new_n139), .d(new_n125), .o1(new_n144));
  xorb03aa1n02x5               g049(.a(new_n144), .b(\b[10] ), .c(\a[11] ), .out0(\s[11] ));
  nor002aa1d32x5               g050(.a(\b[10] ), .b(\a[11] ), .o1(new_n146));
  nand42aa1d28x5               g051(.a(\b[10] ), .b(\a[11] ), .o1(new_n147));
  aoi012aa1n03x5               g052(.a(new_n146), .b(new_n144), .c(new_n147), .o1(new_n148));
  xnrb03aa1n03x5               g053(.a(new_n148), .b(\b[11] ), .c(\a[12] ), .out0(\s[12] ));
  inv000aa1d42x5               g054(.a(new_n117), .o1(new_n150));
  inv000aa1d42x5               g055(.a(new_n118), .o1(new_n151));
  aoai13aa1n06x5               g056(.a(new_n150), .b(new_n151), .c(new_n123), .d(new_n122), .o1(new_n152));
  nor002aa1d32x5               g057(.a(\b[11] ), .b(\a[12] ), .o1(new_n153));
  nanp02aa1n24x5               g058(.a(\b[11] ), .b(\a[12] ), .o1(new_n154));
  nona23aa1n09x5               g059(.a(new_n154), .b(new_n147), .c(new_n146), .d(new_n153), .out0(new_n155));
  nano22aa1n02x4               g060(.a(new_n155), .b(new_n140), .c(new_n127), .out0(new_n156));
  aoai13aa1n06x5               g061(.a(new_n156), .b(new_n152), .c(new_n136), .d(new_n138), .o1(new_n157));
  oao003aa1n02x5               g062(.a(new_n97), .b(new_n142), .c(new_n98), .carry(new_n158));
  nano23aa1d15x5               g063(.a(new_n146), .b(new_n153), .c(new_n154), .d(new_n147), .out0(new_n159));
  tech160nm_fioai012aa1n03p5x5 g064(.a(new_n154), .b(new_n153), .c(new_n146), .o1(new_n160));
  aobi12aa1n06x5               g065(.a(new_n160), .b(new_n159), .c(new_n158), .out0(new_n161));
  nanp02aa1n03x5               g066(.a(new_n157), .b(new_n161), .o1(new_n162));
  xorb03aa1n02x5               g067(.a(new_n162), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  inv000aa1d42x5               g068(.a(\a[13] ), .o1(new_n164));
  inv000aa1d42x5               g069(.a(\b[12] ), .o1(new_n165));
  oaoi03aa1n02x5               g070(.a(new_n164), .b(new_n165), .c(new_n162), .o1(new_n166));
  xnrb03aa1n03x5               g071(.a(new_n166), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  nor002aa1n06x5               g072(.a(\b[12] ), .b(\a[13] ), .o1(new_n168));
  nand42aa1n10x5               g073(.a(\b[12] ), .b(\a[13] ), .o1(new_n169));
  norp02aa1n24x5               g074(.a(\b[13] ), .b(\a[14] ), .o1(new_n170));
  nanp02aa1n12x5               g075(.a(\b[13] ), .b(\a[14] ), .o1(new_n171));
  nona23aa1n03x5               g076(.a(new_n171), .b(new_n169), .c(new_n168), .d(new_n170), .out0(new_n172));
  aoai13aa1n04x5               g077(.a(new_n171), .b(new_n170), .c(new_n164), .d(new_n165), .o1(new_n173));
  aoai13aa1n02x7               g078(.a(new_n173), .b(new_n172), .c(new_n157), .d(new_n161), .o1(new_n174));
  xorb03aa1n02x5               g079(.a(new_n174), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  nor002aa1d32x5               g080(.a(\b[14] ), .b(\a[15] ), .o1(new_n176));
  nand02aa1n16x5               g081(.a(\b[14] ), .b(\a[15] ), .o1(new_n177));
  nanb02aa1d36x5               g082(.a(new_n176), .b(new_n177), .out0(new_n178));
  inv000aa1d42x5               g083(.a(new_n178), .o1(new_n179));
  nor002aa1n10x5               g084(.a(\b[15] ), .b(\a[16] ), .o1(new_n180));
  nand42aa1n04x5               g085(.a(\b[15] ), .b(\a[16] ), .o1(new_n181));
  nanb02aa1n06x5               g086(.a(new_n180), .b(new_n181), .out0(new_n182));
  aoai13aa1n03x5               g087(.a(new_n182), .b(new_n176), .c(new_n174), .d(new_n179), .o1(new_n183));
  nanp02aa1n02x5               g088(.a(new_n174), .b(new_n179), .o1(new_n184));
  nona22aa1n02x4               g089(.a(new_n184), .b(new_n182), .c(new_n176), .out0(new_n185));
  nanp02aa1n03x5               g090(.a(new_n185), .b(new_n183), .o1(\s[16] ));
  nano23aa1n06x5               g091(.a(new_n168), .b(new_n170), .c(new_n171), .d(new_n169), .out0(new_n187));
  nona22aa1n09x5               g092(.a(new_n187), .b(new_n178), .c(new_n182), .out0(new_n188));
  nano32aa1d12x5               g093(.a(new_n188), .b(new_n159), .c(new_n127), .d(new_n140), .out0(new_n189));
  aoai13aa1n12x5               g094(.a(new_n189), .b(new_n152), .c(new_n136), .d(new_n138), .o1(new_n190));
  oaih12aa1n06x5               g095(.a(new_n160), .b(new_n155), .c(new_n143), .o1(new_n191));
  nona23aa1n03x5               g096(.a(new_n181), .b(new_n177), .c(new_n176), .d(new_n180), .out0(new_n192));
  nor042aa1n03x5               g097(.a(new_n192), .b(new_n172), .o1(new_n193));
  oai012aa1n02x5               g098(.a(new_n181), .b(new_n180), .c(new_n176), .o1(new_n194));
  tech160nm_fioai012aa1n05x5   g099(.a(new_n194), .b(new_n192), .c(new_n173), .o1(new_n195));
  aoi012aa1n12x5               g100(.a(new_n195), .b(new_n191), .c(new_n193), .o1(new_n196));
  xorc02aa1n12x5               g101(.a(\a[17] ), .b(\b[16] ), .out0(new_n197));
  xnbna2aa1n03x5               g102(.a(new_n197), .b(new_n190), .c(new_n196), .out0(\s[17] ));
  inv000aa1d42x5               g103(.a(\a[17] ), .o1(new_n199));
  nanb02aa1n12x5               g104(.a(\b[16] ), .b(new_n199), .out0(new_n200));
  oabi12aa1n06x5               g105(.a(new_n195), .b(new_n161), .c(new_n188), .out0(new_n201));
  aoai13aa1n03x5               g106(.a(new_n197), .b(new_n201), .c(new_n126), .d(new_n189), .o1(new_n202));
  xorc02aa1n12x5               g107(.a(\a[18] ), .b(\b[17] ), .out0(new_n203));
  xnbna2aa1n03x5               g108(.a(new_n203), .b(new_n202), .c(new_n200), .out0(\s[18] ));
  inv000aa1d42x5               g109(.a(\a[18] ), .o1(new_n205));
  xroi22aa1d04x5               g110(.a(new_n199), .b(\b[16] ), .c(new_n205), .d(\b[17] ), .out0(new_n206));
  inv000aa1d42x5               g111(.a(new_n206), .o1(new_n207));
  oaih22aa1n06x5               g112(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n208));
  oaib12aa1n09x5               g113(.a(new_n208), .b(new_n205), .c(\b[17] ), .out0(new_n209));
  aoai13aa1n04x5               g114(.a(new_n209), .b(new_n207), .c(new_n190), .d(new_n196), .o1(new_n210));
  xorb03aa1n02x5               g115(.a(new_n210), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g116(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor002aa1d32x5               g117(.a(\b[18] ), .b(\a[19] ), .o1(new_n213));
  nand02aa1d28x5               g118(.a(\b[18] ), .b(\a[19] ), .o1(new_n214));
  nanb02aa1n02x5               g119(.a(new_n213), .b(new_n214), .out0(new_n215));
  inv000aa1d42x5               g120(.a(new_n215), .o1(new_n216));
  nor002aa1d32x5               g121(.a(\b[19] ), .b(\a[20] ), .o1(new_n217));
  nand02aa1d28x5               g122(.a(\b[19] ), .b(\a[20] ), .o1(new_n218));
  nanb02aa1n02x5               g123(.a(new_n217), .b(new_n218), .out0(new_n219));
  aoai13aa1n03x5               g124(.a(new_n219), .b(new_n213), .c(new_n210), .d(new_n216), .o1(new_n220));
  nand02aa1n02x5               g125(.a(new_n156), .b(new_n193), .o1(new_n221));
  aoai13aa1n09x5               g126(.a(new_n196), .b(new_n221), .c(new_n139), .d(new_n125), .o1(new_n222));
  oaoi03aa1n02x5               g127(.a(\a[18] ), .b(\b[17] ), .c(new_n200), .o1(new_n223));
  aoai13aa1n03x5               g128(.a(new_n216), .b(new_n223), .c(new_n222), .d(new_n206), .o1(new_n224));
  nona22aa1n03x5               g129(.a(new_n224), .b(new_n219), .c(new_n213), .out0(new_n225));
  nanp02aa1n03x5               g130(.a(new_n220), .b(new_n225), .o1(\s[20] ));
  nano23aa1n09x5               g131(.a(new_n213), .b(new_n217), .c(new_n218), .d(new_n214), .out0(new_n227));
  nand23aa1n06x5               g132(.a(new_n227), .b(new_n197), .c(new_n203), .o1(new_n228));
  nona23aa1n09x5               g133(.a(new_n218), .b(new_n214), .c(new_n213), .d(new_n217), .out0(new_n229));
  oaih12aa1n06x5               g134(.a(new_n218), .b(new_n217), .c(new_n213), .o1(new_n230));
  oai012aa1n18x5               g135(.a(new_n230), .b(new_n229), .c(new_n209), .o1(new_n231));
  inv000aa1d42x5               g136(.a(new_n231), .o1(new_n232));
  aoai13aa1n04x5               g137(.a(new_n232), .b(new_n228), .c(new_n190), .d(new_n196), .o1(new_n233));
  xorb03aa1n02x5               g138(.a(new_n233), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor002aa1d32x5               g139(.a(\b[20] ), .b(\a[21] ), .o1(new_n235));
  nand22aa1n12x5               g140(.a(\b[20] ), .b(\a[21] ), .o1(new_n236));
  norb02aa1n02x5               g141(.a(new_n236), .b(new_n235), .out0(new_n237));
  nor002aa1d32x5               g142(.a(\b[21] ), .b(\a[22] ), .o1(new_n238));
  nand22aa1n12x5               g143(.a(\b[21] ), .b(\a[22] ), .o1(new_n239));
  norb02aa1n02x5               g144(.a(new_n239), .b(new_n238), .out0(new_n240));
  inv000aa1d42x5               g145(.a(new_n240), .o1(new_n241));
  aoai13aa1n03x5               g146(.a(new_n241), .b(new_n235), .c(new_n233), .d(new_n237), .o1(new_n242));
  inv000aa1d42x5               g147(.a(new_n228), .o1(new_n243));
  aoai13aa1n03x5               g148(.a(new_n237), .b(new_n231), .c(new_n222), .d(new_n243), .o1(new_n244));
  nona22aa1n03x5               g149(.a(new_n244), .b(new_n241), .c(new_n235), .out0(new_n245));
  nanp02aa1n03x5               g150(.a(new_n242), .b(new_n245), .o1(\s[22] ));
  nano23aa1d15x5               g151(.a(new_n235), .b(new_n238), .c(new_n239), .d(new_n236), .out0(new_n247));
  inv000aa1n02x5               g152(.a(new_n247), .o1(new_n248));
  nano32aa1n03x7               g153(.a(new_n248), .b(new_n227), .c(new_n203), .d(new_n197), .out0(new_n249));
  inv000aa1n02x5               g154(.a(new_n249), .o1(new_n250));
  inv030aa1n02x5               g155(.a(new_n239), .o1(new_n251));
  oab012aa1n09x5               g156(.a(new_n251), .b(new_n235), .c(new_n238), .out0(new_n252));
  aoi012aa1d18x5               g157(.a(new_n252), .b(new_n231), .c(new_n247), .o1(new_n253));
  aoai13aa1n04x5               g158(.a(new_n253), .b(new_n250), .c(new_n190), .d(new_n196), .o1(new_n254));
  nor042aa1n04x5               g159(.a(\b[22] ), .b(\a[23] ), .o1(new_n255));
  nand02aa1d16x5               g160(.a(\b[22] ), .b(\a[23] ), .o1(new_n256));
  norb02aa1n02x5               g161(.a(new_n256), .b(new_n255), .out0(new_n257));
  inv000aa1n02x5               g162(.a(new_n253), .o1(new_n258));
  aoi112aa1n02x5               g163(.a(new_n257), .b(new_n258), .c(new_n222), .d(new_n249), .o1(new_n259));
  aoi012aa1n02x5               g164(.a(new_n259), .b(new_n254), .c(new_n257), .o1(\s[23] ));
  nor042aa1n04x5               g165(.a(\b[23] ), .b(\a[24] ), .o1(new_n261));
  nand42aa1d28x5               g166(.a(\b[23] ), .b(\a[24] ), .o1(new_n262));
  norb02aa1n02x5               g167(.a(new_n262), .b(new_n261), .out0(new_n263));
  inv000aa1d42x5               g168(.a(new_n263), .o1(new_n264));
  aoai13aa1n03x5               g169(.a(new_n264), .b(new_n255), .c(new_n254), .d(new_n257), .o1(new_n265));
  aoai13aa1n06x5               g170(.a(new_n257), .b(new_n258), .c(new_n222), .d(new_n249), .o1(new_n266));
  nona22aa1n02x5               g171(.a(new_n266), .b(new_n264), .c(new_n255), .out0(new_n267));
  nanp02aa1n03x5               g172(.a(new_n265), .b(new_n267), .o1(\s[24] ));
  nano23aa1n09x5               g173(.a(new_n255), .b(new_n261), .c(new_n262), .d(new_n256), .out0(new_n269));
  nano22aa1n03x7               g174(.a(new_n228), .b(new_n247), .c(new_n269), .out0(new_n270));
  aoai13aa1n02x5               g175(.a(new_n270), .b(new_n201), .c(new_n126), .d(new_n189), .o1(new_n271));
  inv030aa1n02x5               g176(.a(new_n270), .o1(new_n272));
  nanp02aa1n02x5               g177(.a(new_n227), .b(new_n223), .o1(new_n273));
  nand22aa1n03x5               g178(.a(new_n269), .b(new_n247), .o1(new_n274));
  oai022aa1n06x5               g179(.a(\a[23] ), .b(\b[22] ), .c(\b[23] ), .d(\a[24] ), .o1(new_n275));
  aoi022aa1n12x5               g180(.a(new_n269), .b(new_n252), .c(new_n262), .d(new_n275), .o1(new_n276));
  aoai13aa1n06x5               g181(.a(new_n276), .b(new_n274), .c(new_n273), .d(new_n230), .o1(new_n277));
  inv000aa1n02x5               g182(.a(new_n277), .o1(new_n278));
  aoai13aa1n04x5               g183(.a(new_n278), .b(new_n272), .c(new_n190), .d(new_n196), .o1(new_n279));
  xorc02aa1n12x5               g184(.a(\a[25] ), .b(\b[24] ), .out0(new_n280));
  nanb02aa1n06x5               g185(.a(new_n274), .b(new_n231), .out0(new_n281));
  nano22aa1n02x4               g186(.a(new_n280), .b(new_n281), .c(new_n276), .out0(new_n282));
  aoi022aa1n02x5               g187(.a(new_n279), .b(new_n280), .c(new_n271), .d(new_n282), .o1(\s[25] ));
  norp02aa1n02x5               g188(.a(\b[24] ), .b(\a[25] ), .o1(new_n284));
  xnrc02aa1n12x5               g189(.a(\b[25] ), .b(\a[26] ), .out0(new_n285));
  aoai13aa1n03x5               g190(.a(new_n285), .b(new_n284), .c(new_n279), .d(new_n280), .o1(new_n286));
  aoai13aa1n03x5               g191(.a(new_n280), .b(new_n277), .c(new_n222), .d(new_n270), .o1(new_n287));
  nona22aa1n03x5               g192(.a(new_n287), .b(new_n285), .c(new_n284), .out0(new_n288));
  nanp02aa1n03x5               g193(.a(new_n286), .b(new_n288), .o1(\s[26] ));
  norb02aa1n12x5               g194(.a(new_n280), .b(new_n285), .out0(new_n290));
  nano32aa1n03x7               g195(.a(new_n228), .b(new_n290), .c(new_n247), .d(new_n269), .out0(new_n291));
  aoai13aa1n06x5               g196(.a(new_n291), .b(new_n201), .c(new_n126), .d(new_n189), .o1(new_n292));
  nanp02aa1n02x5               g197(.a(\b[25] ), .b(\a[26] ), .o1(new_n293));
  oai022aa1n02x5               g198(.a(\a[25] ), .b(\b[24] ), .c(\b[25] ), .d(\a[26] ), .o1(new_n294));
  aoi022aa1n12x5               g199(.a(new_n277), .b(new_n290), .c(new_n293), .d(new_n294), .o1(new_n295));
  xorc02aa1n12x5               g200(.a(\a[27] ), .b(\b[26] ), .out0(new_n296));
  xnbna2aa1n03x5               g201(.a(new_n296), .b(new_n292), .c(new_n295), .out0(\s[27] ));
  inv000aa1n02x5               g202(.a(new_n291), .o1(new_n298));
  aoai13aa1n04x5               g203(.a(new_n295), .b(new_n298), .c(new_n190), .d(new_n196), .o1(new_n299));
  nor022aa1n04x5               g204(.a(\b[26] ), .b(\a[27] ), .o1(new_n300));
  norp02aa1n06x5               g205(.a(\b[27] ), .b(\a[28] ), .o1(new_n301));
  nanp02aa1n06x5               g206(.a(\b[27] ), .b(\a[28] ), .o1(new_n302));
  nanb02aa1n06x5               g207(.a(new_n301), .b(new_n302), .out0(new_n303));
  aoai13aa1n03x5               g208(.a(new_n303), .b(new_n300), .c(new_n299), .d(new_n296), .o1(new_n304));
  inv000aa1d42x5               g209(.a(new_n290), .o1(new_n305));
  nanp02aa1n02x5               g210(.a(new_n294), .b(new_n293), .o1(new_n306));
  aoai13aa1n04x5               g211(.a(new_n306), .b(new_n305), .c(new_n281), .d(new_n276), .o1(new_n307));
  aoai13aa1n03x5               g212(.a(new_n296), .b(new_n307), .c(new_n222), .d(new_n291), .o1(new_n308));
  nona22aa1n02x4               g213(.a(new_n308), .b(new_n303), .c(new_n300), .out0(new_n309));
  nanp02aa1n03x5               g214(.a(new_n304), .b(new_n309), .o1(\s[28] ));
  norb02aa1n03x5               g215(.a(new_n296), .b(new_n303), .out0(new_n311));
  aoai13aa1n03x5               g216(.a(new_n311), .b(new_n307), .c(new_n222), .d(new_n291), .o1(new_n312));
  inv000aa1d42x5               g217(.a(new_n311), .o1(new_n313));
  oai012aa1n02x5               g218(.a(new_n302), .b(new_n301), .c(new_n300), .o1(new_n314));
  aoai13aa1n03x5               g219(.a(new_n314), .b(new_n313), .c(new_n292), .d(new_n295), .o1(new_n315));
  norp02aa1n02x5               g220(.a(\b[28] ), .b(\a[29] ), .o1(new_n316));
  nanp02aa1n02x5               g221(.a(\b[28] ), .b(\a[29] ), .o1(new_n317));
  norb02aa1n02x5               g222(.a(new_n317), .b(new_n316), .out0(new_n318));
  oai022aa1n02x5               g223(.a(\a[27] ), .b(\b[26] ), .c(\b[27] ), .d(\a[28] ), .o1(new_n319));
  aboi22aa1n03x5               g224(.a(new_n316), .b(new_n317), .c(new_n319), .d(new_n302), .out0(new_n320));
  aoi022aa1n02x7               g225(.a(new_n315), .b(new_n318), .c(new_n312), .d(new_n320), .o1(\s[29] ));
  xnrb03aa1n02x5               g226(.a(new_n99), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nanb03aa1n02x5               g227(.a(new_n303), .b(new_n296), .c(new_n318), .out0(new_n323));
  nanb02aa1n03x5               g228(.a(new_n323), .b(new_n299), .out0(new_n324));
  oaoi03aa1n02x5               g229(.a(\a[29] ), .b(\b[28] ), .c(new_n314), .o1(new_n325));
  inv000aa1n03x5               g230(.a(new_n325), .o1(new_n326));
  aoai13aa1n02x5               g231(.a(new_n326), .b(new_n323), .c(new_n292), .d(new_n295), .o1(new_n327));
  xorc02aa1n02x5               g232(.a(\a[30] ), .b(\b[29] ), .out0(new_n328));
  aoi113aa1n02x5               g233(.a(new_n328), .b(new_n316), .c(new_n319), .d(new_n317), .e(new_n302), .o1(new_n329));
  aoi022aa1n03x5               g234(.a(new_n327), .b(new_n328), .c(new_n324), .d(new_n329), .o1(\s[30] ));
  nand03aa1n02x5               g235(.a(new_n311), .b(new_n318), .c(new_n328), .o1(new_n331));
  nanb02aa1n03x5               g236(.a(new_n331), .b(new_n299), .out0(new_n332));
  xorc02aa1n02x5               g237(.a(\a[31] ), .b(\b[30] ), .out0(new_n333));
  oao003aa1n02x5               g238(.a(\a[30] ), .b(\b[29] ), .c(new_n326), .carry(new_n334));
  norb02aa1n02x5               g239(.a(new_n334), .b(new_n333), .out0(new_n335));
  aoai13aa1n02x5               g240(.a(new_n334), .b(new_n331), .c(new_n292), .d(new_n295), .o1(new_n336));
  aoi022aa1n03x5               g241(.a(new_n336), .b(new_n333), .c(new_n332), .d(new_n335), .o1(\s[31] ));
  xnrb03aa1n02x5               g242(.a(new_n134), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  aoi122aa1n02x5               g243(.a(new_n103), .b(new_n106), .c(new_n102), .d(new_n100), .e(new_n104), .o1(new_n339));
  aoi012aa1n02x5               g244(.a(new_n339), .b(new_n106), .c(new_n136), .o1(\s[4] ));
  xorb03aa1n02x5               g245(.a(new_n136), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  oaoi03aa1n02x5               g246(.a(\a[5] ), .b(\b[4] ), .c(new_n108), .o1(new_n342));
  xnrc02aa1n02x5               g247(.a(new_n342), .b(new_n114), .out0(\s[6] ));
  inv000aa1d42x5               g248(.a(new_n113), .o1(new_n344));
  nanp02aa1n02x5               g249(.a(new_n122), .b(new_n116), .o1(new_n345));
  aoi112aa1n06x5               g250(.a(new_n109), .b(new_n114), .c(new_n136), .d(new_n110), .o1(new_n346));
  nano32aa1n03x7               g251(.a(new_n346), .b(new_n116), .c(new_n122), .d(new_n113), .out0(new_n347));
  oaoi13aa1n02x5               g252(.a(new_n347), .b(new_n345), .c(new_n344), .d(new_n346), .o1(\s[7] ));
  nor042aa1n03x5               g253(.a(new_n347), .b(new_n115), .o1(new_n349));
  xnbna2aa1n03x5               g254(.a(new_n349), .b(new_n150), .c(new_n118), .out0(\s[8] ));
  xnbna2aa1n03x5               g255(.a(new_n127), .b(new_n139), .c(new_n125), .out0(\s[9] ));
endmodule


