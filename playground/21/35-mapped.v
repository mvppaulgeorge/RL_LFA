// Benchmark "adder" written by ABC on Wed Jul 17 23:02:22 2024

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
    new_n132, new_n133, new_n134, new_n135, new_n136, new_n137, new_n138,
    new_n139, new_n140, new_n142, new_n143, new_n144, new_n145, new_n146,
    new_n148, new_n149, new_n150, new_n151, new_n152, new_n153, new_n155,
    new_n156, new_n157, new_n158, new_n159, new_n160, new_n161, new_n162,
    new_n163, new_n164, new_n165, new_n166, new_n167, new_n168, new_n170,
    new_n171, new_n172, new_n173, new_n174, new_n175, new_n176, new_n177,
    new_n180, new_n181, new_n182, new_n183, new_n184, new_n185, new_n186,
    new_n187, new_n188, new_n189, new_n190, new_n191, new_n192, new_n193,
    new_n194, new_n196, new_n198, new_n199, new_n200, new_n201, new_n202,
    new_n203, new_n204, new_n205, new_n206, new_n207, new_n208, new_n209,
    new_n210, new_n211, new_n212, new_n213, new_n215, new_n216, new_n217,
    new_n220, new_n221, new_n222, new_n223, new_n224, new_n225, new_n226,
    new_n227, new_n228, new_n230, new_n231, new_n232, new_n233, new_n234,
    new_n235, new_n236, new_n237, new_n238, new_n240, new_n241, new_n242,
    new_n243, new_n244, new_n245, new_n247, new_n248, new_n249, new_n250,
    new_n251, new_n252, new_n253, new_n254, new_n255, new_n256, new_n257,
    new_n258, new_n260, new_n261, new_n262, new_n263, new_n264, new_n265,
    new_n267, new_n268, new_n269, new_n270, new_n271, new_n272, new_n273,
    new_n274, new_n276, new_n277, new_n278, new_n279, new_n280, new_n281,
    new_n283, new_n284, new_n285, new_n286, new_n287, new_n288, new_n290,
    new_n291, new_n292, new_n293, new_n294, new_n295, new_n296, new_n297,
    new_n298, new_n299, new_n300, new_n302, new_n303, new_n304, new_n305,
    new_n306, new_n307, new_n308, new_n309, new_n310, new_n313, new_n314,
    new_n315, new_n316, new_n317, new_n318, new_n319, new_n321, new_n322,
    new_n323, new_n324, new_n325, new_n326, new_n327, new_n328, new_n329,
    new_n330, new_n331, new_n333, new_n335, new_n337, new_n339, new_n341,
    new_n343;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor042aa1n06x5               g001(.a(\b[9] ), .b(\a[10] ), .o1(new_n97));
  nand02aa1n04x5               g002(.a(\b[9] ), .b(\a[10] ), .o1(new_n98));
  nanb02aa1n02x5               g003(.a(new_n97), .b(new_n98), .out0(new_n99));
  nor042aa1n04x5               g004(.a(\b[8] ), .b(\a[9] ), .o1(new_n100));
  nand02aa1n03x5               g005(.a(\b[8] ), .b(\a[9] ), .o1(new_n101));
  nor042aa1n06x5               g006(.a(\b[3] ), .b(\a[4] ), .o1(new_n102));
  nand42aa1n10x5               g007(.a(\b[3] ), .b(\a[4] ), .o1(new_n103));
  norb02aa1d21x5               g008(.a(new_n103), .b(new_n102), .out0(new_n104));
  nand22aa1n12x5               g009(.a(\b[0] ), .b(\a[1] ), .o1(new_n105));
  nand42aa1n20x5               g010(.a(\b[1] ), .b(\a[2] ), .o1(new_n106));
  nor002aa1d32x5               g011(.a(\b[1] ), .b(\a[2] ), .o1(new_n107));
  nona22aa1n09x5               g012(.a(new_n106), .b(new_n107), .c(new_n105), .out0(new_n108));
  nor042aa1n06x5               g013(.a(\b[2] ), .b(\a[3] ), .o1(new_n109));
  nand42aa1d28x5               g014(.a(\b[2] ), .b(\a[3] ), .o1(new_n110));
  nano22aa1d15x5               g015(.a(new_n109), .b(new_n106), .c(new_n110), .out0(new_n111));
  aoi012aa1n06x5               g016(.a(new_n102), .b(new_n109), .c(new_n103), .o1(new_n112));
  inv000aa1n02x5               g017(.a(new_n112), .o1(new_n113));
  aoi013aa1n06x4               g018(.a(new_n113), .b(new_n111), .c(new_n108), .d(new_n104), .o1(new_n114));
  norp02aa1n04x5               g019(.a(\b[7] ), .b(\a[8] ), .o1(new_n115));
  nand02aa1d24x5               g020(.a(\b[7] ), .b(\a[8] ), .o1(new_n116));
  nor042aa1n09x5               g021(.a(\b[6] ), .b(\a[7] ), .o1(new_n117));
  nand02aa1n10x5               g022(.a(\b[6] ), .b(\a[7] ), .o1(new_n118));
  nano23aa1n06x5               g023(.a(new_n115), .b(new_n117), .c(new_n118), .d(new_n116), .out0(new_n119));
  nor022aa1n16x5               g024(.a(\b[5] ), .b(\a[6] ), .o1(new_n120));
  nand42aa1d28x5               g025(.a(\b[5] ), .b(\a[6] ), .o1(new_n121));
  norb02aa1n09x5               g026(.a(new_n121), .b(new_n120), .out0(new_n122));
  xorc02aa1n12x5               g027(.a(\a[5] ), .b(\b[4] ), .out0(new_n123));
  nanp03aa1d12x5               g028(.a(new_n119), .b(new_n122), .c(new_n123), .o1(new_n124));
  orn002aa1n24x5               g029(.a(\a[5] ), .b(\b[4] ), .o(new_n125));
  nanb03aa1d18x5               g030(.a(new_n120), .b(new_n125), .c(new_n121), .out0(new_n126));
  inv000aa1d42x5               g031(.a(\b[7] ), .o1(new_n127));
  nanb02aa1n03x5               g032(.a(\a[8] ), .b(new_n127), .out0(new_n128));
  nanb02aa1n03x5               g033(.a(new_n117), .b(new_n118), .out0(new_n129));
  nano32aa1n03x7               g034(.a(new_n129), .b(new_n128), .c(new_n121), .d(new_n116), .out0(new_n130));
  aoi012aa1n02x5               g035(.a(new_n115), .b(new_n117), .c(new_n116), .o1(new_n131));
  aobi12aa1n06x5               g036(.a(new_n131), .b(new_n130), .c(new_n126), .out0(new_n132));
  tech160nm_fioai012aa1n05x5   g037(.a(new_n132), .b(new_n114), .c(new_n124), .o1(new_n133));
  aoai13aa1n02x5               g038(.a(new_n99), .b(new_n100), .c(new_n133), .d(new_n101), .o1(new_n134));
  nanp03aa1n06x5               g039(.a(new_n111), .b(new_n108), .c(new_n104), .o1(new_n135));
  nand02aa1d06x5               g040(.a(new_n135), .b(new_n112), .o1(new_n136));
  inv040aa1n04x5               g041(.a(new_n124), .o1(new_n137));
  aob012aa1n03x5               g042(.a(new_n131), .b(new_n130), .c(new_n126), .out0(new_n138));
  aoai13aa1n02x5               g043(.a(new_n101), .b(new_n138), .c(new_n137), .d(new_n136), .o1(new_n139));
  nona22aa1n02x4               g044(.a(new_n139), .b(new_n100), .c(new_n99), .out0(new_n140));
  nanp02aa1n02x5               g045(.a(new_n134), .b(new_n140), .o1(\s[10] ));
  nor042aa1d18x5               g046(.a(\b[10] ), .b(\a[11] ), .o1(new_n142));
  inv040aa1n08x5               g047(.a(new_n142), .o1(new_n143));
  aoi022aa1n02x5               g048(.a(\b[9] ), .b(\a[10] ), .c(\a[11] ), .d(\b[10] ), .o1(new_n144));
  nand02aa1d24x5               g049(.a(\b[10] ), .b(\a[11] ), .o1(new_n145));
  aoi022aa1n02x5               g050(.a(new_n140), .b(new_n98), .c(new_n145), .d(new_n143), .o1(new_n146));
  aoi013aa1n02x4               g051(.a(new_n146), .b(new_n144), .c(new_n143), .d(new_n140), .o1(\s[11] ));
  nor042aa1n04x5               g052(.a(\b[11] ), .b(\a[12] ), .o1(new_n148));
  nand02aa1n10x5               g053(.a(\b[11] ), .b(\a[12] ), .o1(new_n149));
  norb02aa1n12x5               g054(.a(new_n149), .b(new_n148), .out0(new_n150));
  inv000aa1d42x5               g055(.a(\a[11] ), .o1(new_n151));
  inv000aa1d42x5               g056(.a(\b[10] ), .o1(new_n152));
  aoi022aa1n02x5               g057(.a(new_n140), .b(new_n144), .c(new_n152), .d(new_n151), .o1(new_n153));
  xnrc02aa1n02x5               g058(.a(new_n153), .b(new_n150), .out0(\s[12] ));
  norb02aa1n12x5               g059(.a(new_n145), .b(new_n142), .out0(new_n155));
  nona23aa1n12x5               g060(.a(new_n101), .b(new_n98), .c(new_n97), .d(new_n100), .out0(new_n156));
  nano22aa1n03x7               g061(.a(new_n156), .b(new_n155), .c(new_n150), .out0(new_n157));
  aoi112aa1n09x5               g062(.a(\b[8] ), .b(\a[9] ), .c(\a[10] ), .d(\b[9] ), .o1(new_n158));
  oai112aa1n06x5               g063(.a(new_n150), .b(new_n155), .c(new_n158), .d(new_n97), .o1(new_n159));
  oaoi03aa1n09x5               g064(.a(\a[12] ), .b(\b[11] ), .c(new_n143), .o1(new_n160));
  inv040aa1n03x5               g065(.a(new_n160), .o1(new_n161));
  nanp02aa1n02x5               g066(.a(new_n159), .b(new_n161), .o1(new_n162));
  nor042aa1n04x5               g067(.a(\b[12] ), .b(\a[13] ), .o1(new_n163));
  nand02aa1d08x5               g068(.a(\b[12] ), .b(\a[13] ), .o1(new_n164));
  nanb02aa1n02x5               g069(.a(new_n163), .b(new_n164), .out0(new_n165));
  inv000aa1d42x5               g070(.a(new_n165), .o1(new_n166));
  aoai13aa1n02x5               g071(.a(new_n166), .b(new_n162), .c(new_n133), .d(new_n157), .o1(new_n167));
  aoi112aa1n02x5               g072(.a(new_n166), .b(new_n162), .c(new_n133), .d(new_n157), .o1(new_n168));
  norb02aa1n02x5               g073(.a(new_n167), .b(new_n168), .out0(\s[13] ));
  nor042aa1n04x5               g074(.a(\b[13] ), .b(\a[14] ), .o1(new_n170));
  nand02aa1d08x5               g075(.a(\b[13] ), .b(\a[14] ), .o1(new_n171));
  aoib12aa1n02x5               g076(.a(new_n163), .b(new_n171), .c(new_n170), .out0(new_n172));
  nona23aa1n09x5               g077(.a(new_n171), .b(new_n164), .c(new_n163), .d(new_n170), .out0(new_n173));
  nano23aa1n02x4               g078(.a(new_n173), .b(new_n156), .c(new_n150), .d(new_n155), .out0(new_n174));
  oai012aa1n02x5               g079(.a(new_n171), .b(new_n170), .c(new_n163), .o1(new_n175));
  aoai13aa1n06x5               g080(.a(new_n175), .b(new_n173), .c(new_n159), .d(new_n161), .o1(new_n176));
  tech160nm_fiao0012aa1n03p5x5 g081(.a(new_n176), .b(new_n133), .c(new_n174), .o(new_n177));
  aboi22aa1n03x5               g082(.a(new_n170), .b(new_n177), .c(new_n167), .d(new_n172), .out0(\s[14] ));
  xorb03aa1n02x5               g083(.a(new_n177), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  nor002aa1d32x5               g084(.a(\b[14] ), .b(\a[15] ), .o1(new_n180));
  nanp02aa1n04x5               g085(.a(\b[14] ), .b(\a[15] ), .o1(new_n181));
  nanb03aa1n03x5               g086(.a(new_n180), .b(new_n177), .c(new_n181), .out0(new_n182));
  nor002aa1d32x5               g087(.a(\b[15] ), .b(\a[16] ), .o1(new_n183));
  nanp02aa1n04x5               g088(.a(\b[15] ), .b(\a[16] ), .o1(new_n184));
  aoib12aa1n02x5               g089(.a(new_n180), .b(new_n184), .c(new_n183), .out0(new_n185));
  nano23aa1n03x7               g090(.a(new_n142), .b(new_n148), .c(new_n149), .d(new_n145), .out0(new_n186));
  nano23aa1n03x7               g091(.a(new_n163), .b(new_n170), .c(new_n171), .d(new_n164), .out0(new_n187));
  nona23aa1d16x5               g092(.a(new_n184), .b(new_n181), .c(new_n180), .d(new_n183), .out0(new_n188));
  nano23aa1n06x5               g093(.a(new_n188), .b(new_n156), .c(new_n186), .d(new_n187), .out0(new_n189));
  aoai13aa1n06x5               g094(.a(new_n189), .b(new_n138), .c(new_n137), .d(new_n136), .o1(new_n190));
  inv000aa1d42x5               g095(.a(new_n188), .o1(new_n191));
  oai012aa1n02x5               g096(.a(new_n184), .b(new_n183), .c(new_n180), .o1(new_n192));
  aobi12aa1n09x5               g097(.a(new_n192), .b(new_n176), .c(new_n191), .out0(new_n193));
  nanp02aa1n12x5               g098(.a(new_n193), .b(new_n190), .o1(new_n194));
  aboi22aa1n03x5               g099(.a(new_n183), .b(new_n194), .c(new_n182), .d(new_n185), .out0(\s[16] ));
  xorc02aa1n12x5               g100(.a(\a[17] ), .b(\b[16] ), .out0(new_n196));
  xnbna2aa1n03x5               g101(.a(new_n196), .b(new_n193), .c(new_n190), .out0(\s[17] ));
  nona22aa1n09x5               g102(.a(new_n157), .b(new_n173), .c(new_n188), .out0(new_n198));
  oaoi13aa1n09x5               g103(.a(new_n198), .b(new_n132), .c(new_n114), .d(new_n124), .o1(new_n199));
  tech160nm_fiao0012aa1n02p5x5 g104(.a(new_n97), .b(new_n100), .c(new_n98), .o(new_n200));
  aoai13aa1n02x7               g105(.a(new_n187), .b(new_n160), .c(new_n186), .d(new_n200), .o1(new_n201));
  aoai13aa1n03x5               g106(.a(new_n192), .b(new_n188), .c(new_n201), .d(new_n175), .o1(new_n202));
  inv040aa1d32x5               g107(.a(\a[17] ), .o1(new_n203));
  inv000aa1d42x5               g108(.a(\b[16] ), .o1(new_n204));
  nanp02aa1n02x5               g109(.a(new_n204), .b(new_n203), .o1(new_n205));
  nor042aa1n06x5               g110(.a(\b[17] ), .b(\a[18] ), .o1(new_n206));
  nanp02aa1n04x5               g111(.a(\b[17] ), .b(\a[18] ), .o1(new_n207));
  oaib12aa1n02x5               g112(.a(new_n205), .b(new_n206), .c(new_n207), .out0(new_n208));
  oaoi13aa1n02x5               g113(.a(new_n208), .b(new_n196), .c(new_n202), .d(new_n199), .o1(new_n209));
  nano22aa1d15x5               g114(.a(new_n206), .b(new_n196), .c(new_n207), .out0(new_n210));
  oaih12aa1n02x5               g115(.a(new_n210), .b(new_n202), .c(new_n199), .o1(new_n211));
  aoai13aa1n06x5               g116(.a(new_n207), .b(new_n206), .c(new_n203), .d(new_n204), .o1(new_n212));
  aoi012aa1n02x5               g117(.a(new_n206), .b(new_n211), .c(new_n212), .o1(new_n213));
  norp02aa1n02x5               g118(.a(new_n213), .b(new_n209), .o1(\s[18] ));
  nor002aa1d32x5               g119(.a(\b[18] ), .b(\a[19] ), .o1(new_n215));
  nand42aa1n10x5               g120(.a(\b[18] ), .b(\a[19] ), .o1(new_n216));
  norb02aa1n02x5               g121(.a(new_n216), .b(new_n215), .out0(new_n217));
  xnbna2aa1n03x5               g122(.a(new_n217), .b(new_n211), .c(new_n212), .out0(\s[19] ));
  xnrc02aa1n02x5               g123(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  inv000aa1d42x5               g124(.a(new_n215), .o1(new_n220));
  oaoi03aa1n12x5               g125(.a(\a[18] ), .b(\b[17] ), .c(new_n205), .o1(new_n221));
  aoai13aa1n06x5               g126(.a(new_n217), .b(new_n221), .c(new_n194), .d(new_n210), .o1(new_n222));
  nor002aa1d32x5               g127(.a(\b[19] ), .b(\a[20] ), .o1(new_n223));
  nand02aa1d16x5               g128(.a(\b[19] ), .b(\a[20] ), .o1(new_n224));
  norb02aa1n02x5               g129(.a(new_n224), .b(new_n223), .out0(new_n225));
  inv020aa1n04x5               g130(.a(new_n224), .o1(new_n226));
  oai022aa1n02x5               g131(.a(\a[19] ), .b(\b[18] ), .c(\b[19] ), .d(\a[20] ), .o1(new_n227));
  nona22aa1n03x5               g132(.a(new_n222), .b(new_n226), .c(new_n227), .out0(new_n228));
  aoai13aa1n03x5               g133(.a(new_n228), .b(new_n225), .c(new_n220), .d(new_n222), .o1(\s[20] ));
  nona23aa1d18x5               g134(.a(new_n224), .b(new_n216), .c(new_n215), .d(new_n223), .out0(new_n230));
  nano23aa1n02x4               g135(.a(new_n230), .b(new_n206), .c(new_n196), .d(new_n207), .out0(new_n231));
  oab012aa1n03x5               g136(.a(new_n226), .b(new_n215), .c(new_n223), .out0(new_n232));
  oabi12aa1n06x5               g137(.a(new_n232), .b(new_n230), .c(new_n212), .out0(new_n233));
  nor042aa1n09x5               g138(.a(\b[20] ), .b(\a[21] ), .o1(new_n234));
  nanp02aa1n02x5               g139(.a(\b[20] ), .b(\a[21] ), .o1(new_n235));
  norb02aa1n02x5               g140(.a(new_n235), .b(new_n234), .out0(new_n236));
  aoai13aa1n06x5               g141(.a(new_n236), .b(new_n233), .c(new_n194), .d(new_n231), .o1(new_n237));
  aoi112aa1n02x5               g142(.a(new_n236), .b(new_n233), .c(new_n194), .d(new_n231), .o1(new_n238));
  norb02aa1n03x4               g143(.a(new_n237), .b(new_n238), .out0(\s[21] ));
  inv000aa1d42x5               g144(.a(new_n234), .o1(new_n240));
  xnrc02aa1n12x5               g145(.a(\b[21] ), .b(\a[22] ), .out0(new_n241));
  inv000aa1d42x5               g146(.a(new_n241), .o1(new_n242));
  and002aa1n02x5               g147(.a(\b[21] ), .b(\a[22] ), .o(new_n243));
  oai022aa1n02x5               g148(.a(\a[21] ), .b(\b[20] ), .c(\b[21] ), .d(\a[22] ), .o1(new_n244));
  nona22aa1n03x5               g149(.a(new_n237), .b(new_n243), .c(new_n244), .out0(new_n245));
  aoai13aa1n03x5               g150(.a(new_n245), .b(new_n242), .c(new_n240), .d(new_n237), .o1(\s[22] ));
  nano23aa1d15x5               g151(.a(new_n215), .b(new_n223), .c(new_n224), .d(new_n216), .out0(new_n247));
  nano22aa1d15x5               g152(.a(new_n241), .b(new_n240), .c(new_n235), .out0(new_n248));
  nand23aa1n09x5               g153(.a(new_n210), .b(new_n248), .c(new_n247), .o1(new_n249));
  inv000aa1d42x5               g154(.a(new_n249), .o1(new_n250));
  aoai13aa1n06x5               g155(.a(new_n248), .b(new_n232), .c(new_n247), .d(new_n221), .o1(new_n251));
  oaoi03aa1n09x5               g156(.a(\a[22] ), .b(\b[21] ), .c(new_n240), .o1(new_n252));
  inv000aa1d42x5               g157(.a(new_n252), .o1(new_n253));
  nanp02aa1n02x5               g158(.a(new_n251), .b(new_n253), .o1(new_n254));
  xnrc02aa1n12x5               g159(.a(\b[22] ), .b(\a[23] ), .out0(new_n255));
  inv000aa1d42x5               g160(.a(new_n255), .o1(new_n256));
  aoai13aa1n06x5               g161(.a(new_n256), .b(new_n254), .c(new_n194), .d(new_n250), .o1(new_n257));
  aoi112aa1n02x5               g162(.a(new_n256), .b(new_n254), .c(new_n194), .d(new_n250), .o1(new_n258));
  norb02aa1n03x4               g163(.a(new_n257), .b(new_n258), .out0(\s[23] ));
  norp02aa1n02x5               g164(.a(\b[22] ), .b(\a[23] ), .o1(new_n260));
  inv000aa1d42x5               g165(.a(new_n260), .o1(new_n261));
  xorc02aa1n02x5               g166(.a(\a[24] ), .b(\b[23] ), .out0(new_n262));
  and002aa1n02x5               g167(.a(\b[23] ), .b(\a[24] ), .o(new_n263));
  oai022aa1n02x5               g168(.a(\a[23] ), .b(\b[22] ), .c(\b[23] ), .d(\a[24] ), .o1(new_n264));
  nona22aa1n03x5               g169(.a(new_n257), .b(new_n263), .c(new_n264), .out0(new_n265));
  aoai13aa1n03x5               g170(.a(new_n265), .b(new_n262), .c(new_n261), .d(new_n257), .o1(\s[24] ));
  norb02aa1n06x4               g171(.a(new_n262), .b(new_n255), .out0(new_n267));
  inv000aa1n02x5               g172(.a(new_n267), .o1(new_n268));
  nano32aa1n02x4               g173(.a(new_n268), .b(new_n210), .c(new_n248), .d(new_n247), .out0(new_n269));
  aob012aa1n02x5               g174(.a(new_n264), .b(\b[23] ), .c(\a[24] ), .out0(new_n270));
  aoai13aa1n06x5               g175(.a(new_n270), .b(new_n268), .c(new_n251), .d(new_n253), .o1(new_n271));
  xorc02aa1n12x5               g176(.a(\a[25] ), .b(\b[24] ), .out0(new_n272));
  aoai13aa1n06x5               g177(.a(new_n272), .b(new_n271), .c(new_n194), .d(new_n269), .o1(new_n273));
  aoi112aa1n02x5               g178(.a(new_n272), .b(new_n271), .c(new_n194), .d(new_n269), .o1(new_n274));
  norb02aa1n03x4               g179(.a(new_n273), .b(new_n274), .out0(\s[25] ));
  norp02aa1n02x5               g180(.a(\b[24] ), .b(\a[25] ), .o1(new_n276));
  inv000aa1d42x5               g181(.a(new_n276), .o1(new_n277));
  tech160nm_fixorc02aa1n05x5   g182(.a(\a[26] ), .b(\b[25] ), .out0(new_n278));
  and002aa1n02x5               g183(.a(\b[25] ), .b(\a[26] ), .o(new_n279));
  oai022aa1n02x5               g184(.a(\a[25] ), .b(\b[24] ), .c(\b[25] ), .d(\a[26] ), .o1(new_n280));
  nona22aa1n03x5               g185(.a(new_n273), .b(new_n279), .c(new_n280), .out0(new_n281));
  aoai13aa1n03x5               g186(.a(new_n281), .b(new_n278), .c(new_n277), .d(new_n273), .o1(\s[26] ));
  and002aa1n06x5               g187(.a(new_n278), .b(new_n272), .o(new_n283));
  nano22aa1d15x5               g188(.a(new_n249), .b(new_n283), .c(new_n267), .out0(new_n284));
  oai012aa1n06x5               g189(.a(new_n284), .b(new_n202), .c(new_n199), .o1(new_n285));
  inv000aa1d42x5               g190(.a(new_n279), .o1(new_n286));
  aoi022aa1n06x5               g191(.a(new_n271), .b(new_n283), .c(new_n286), .d(new_n280), .o1(new_n287));
  xorc02aa1n12x5               g192(.a(\a[27] ), .b(\b[26] ), .out0(new_n288));
  xnbna2aa1n03x5               g193(.a(new_n288), .b(new_n285), .c(new_n287), .out0(\s[27] ));
  nor042aa1n03x5               g194(.a(\b[26] ), .b(\a[27] ), .o1(new_n290));
  inv040aa1n03x5               g195(.a(new_n290), .o1(new_n291));
  aoai13aa1n06x5               g196(.a(new_n267), .b(new_n252), .c(new_n233), .d(new_n248), .o1(new_n292));
  inv000aa1d42x5               g197(.a(new_n283), .o1(new_n293));
  aob012aa1n02x5               g198(.a(new_n280), .b(\b[25] ), .c(\a[26] ), .out0(new_n294));
  aoai13aa1n06x5               g199(.a(new_n294), .b(new_n293), .c(new_n292), .d(new_n270), .o1(new_n295));
  aoai13aa1n06x5               g200(.a(new_n288), .b(new_n295), .c(new_n194), .d(new_n284), .o1(new_n296));
  xorc02aa1n02x5               g201(.a(\a[28] ), .b(\b[27] ), .out0(new_n297));
  oai022aa1n02x5               g202(.a(\a[27] ), .b(\b[26] ), .c(\b[27] ), .d(\a[28] ), .o1(new_n298));
  aoi012aa1n02x5               g203(.a(new_n298), .b(\a[28] ), .c(\b[27] ), .o1(new_n299));
  nand42aa1n02x5               g204(.a(new_n296), .b(new_n299), .o1(new_n300));
  aoai13aa1n03x5               g205(.a(new_n300), .b(new_n297), .c(new_n291), .d(new_n296), .o1(\s[28] ));
  xorc02aa1n12x5               g206(.a(\a[29] ), .b(\b[28] ), .out0(new_n302));
  and002aa1n02x5               g207(.a(new_n297), .b(new_n288), .o(new_n303));
  aoai13aa1n04x5               g208(.a(new_n303), .b(new_n295), .c(new_n194), .d(new_n284), .o1(new_n304));
  tech160nm_fioaoi03aa1n03p5x5 g209(.a(\a[28] ), .b(\b[27] ), .c(new_n291), .o1(new_n305));
  inv000aa1n03x5               g210(.a(new_n305), .o1(new_n306));
  nand03aa1n02x5               g211(.a(new_n304), .b(new_n302), .c(new_n306), .o1(new_n307));
  inv040aa1n06x5               g212(.a(new_n284), .o1(new_n308));
  tech160nm_fiaoi012aa1n03p5x5 g213(.a(new_n308), .b(new_n193), .c(new_n190), .o1(new_n309));
  oaoi13aa1n06x5               g214(.a(new_n305), .b(new_n303), .c(new_n309), .d(new_n295), .o1(new_n310));
  tech160nm_fioai012aa1n02p5x5 g215(.a(new_n307), .b(new_n310), .c(new_n302), .o1(\s[29] ));
  xorb03aa1n02x5               g216(.a(new_n105), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  and003aa1n06x5               g217(.a(new_n288), .b(new_n302), .c(new_n297), .o(new_n313));
  tech160nm_fioaoi03aa1n03p5x5 g218(.a(\a[29] ), .b(\b[28] ), .c(new_n306), .o1(new_n314));
  oaoi13aa1n06x5               g219(.a(new_n314), .b(new_n313), .c(new_n309), .d(new_n295), .o1(new_n315));
  xorc02aa1n02x5               g220(.a(\a[30] ), .b(\b[29] ), .out0(new_n316));
  inv000aa1d42x5               g221(.a(new_n313), .o1(new_n317));
  norb02aa1n02x5               g222(.a(new_n316), .b(new_n314), .out0(new_n318));
  aoai13aa1n03x5               g223(.a(new_n318), .b(new_n317), .c(new_n285), .d(new_n287), .o1(new_n319));
  oai012aa1n03x5               g224(.a(new_n319), .b(new_n315), .c(new_n316), .o1(\s[30] ));
  and003aa1d24x5               g225(.a(new_n303), .b(new_n316), .c(new_n302), .o(new_n321));
  inv000aa1d42x5               g226(.a(new_n321), .o1(new_n322));
  inv000aa1d42x5               g227(.a(\a[30] ), .o1(new_n323));
  inv000aa1d42x5               g228(.a(\b[29] ), .o1(new_n324));
  oaoi03aa1n12x5               g229(.a(new_n323), .b(new_n324), .c(new_n314), .o1(new_n325));
  aoai13aa1n06x5               g230(.a(new_n325), .b(new_n322), .c(new_n285), .d(new_n287), .o1(new_n326));
  xnrc02aa1n02x5               g231(.a(\b[30] ), .b(\a[31] ), .out0(new_n327));
  nanp02aa1n03x5               g232(.a(new_n326), .b(new_n327), .o1(new_n328));
  aoai13aa1n06x5               g233(.a(new_n321), .b(new_n295), .c(new_n194), .d(new_n284), .o1(new_n329));
  inv000aa1n02x5               g234(.a(new_n325), .o1(new_n330));
  nona22aa1n02x4               g235(.a(new_n329), .b(new_n330), .c(new_n327), .out0(new_n331));
  nanp02aa1n03x5               g236(.a(new_n328), .b(new_n331), .o1(\s[31] ));
  norb02aa1n02x5               g237(.a(new_n110), .b(new_n109), .out0(new_n333));
  xobna2aa1n03x5               g238(.a(new_n333), .b(new_n108), .c(new_n106), .out0(\s[3] ));
  aoi012aa1n02x5               g239(.a(new_n109), .b(new_n111), .c(new_n108), .o1(new_n335));
  xnrc02aa1n02x5               g240(.a(new_n335), .b(new_n104), .out0(\s[4] ));
  aoi112aa1n02x5               g241(.a(new_n123), .b(new_n102), .c(new_n103), .d(new_n109), .o1(new_n337));
  aoi022aa1n02x5               g242(.a(new_n136), .b(new_n123), .c(new_n135), .d(new_n337), .o1(\s[5] ));
  nanp02aa1n02x5               g243(.a(new_n136), .b(new_n123), .o1(new_n339));
  xnbna2aa1n03x5               g244(.a(new_n122), .b(new_n339), .c(new_n125), .out0(\s[6] ));
  aoai13aa1n02x5               g245(.a(new_n121), .b(new_n126), .c(new_n136), .d(new_n123), .o1(new_n341));
  xnrb03aa1n02x5               g246(.a(new_n341), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  oaoi03aa1n02x5               g247(.a(\a[7] ), .b(\b[6] ), .c(new_n341), .o1(new_n343));
  xorb03aa1n02x5               g248(.a(new_n343), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xorb03aa1n02x5               g249(.a(new_n133), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


