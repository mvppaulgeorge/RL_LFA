// Benchmark "adder" written by ABC on Thu Jul 18 02:03:46 2024

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
    new_n125, new_n126, new_n127, new_n129, new_n130, new_n131, new_n132,
    new_n133, new_n134, new_n135, new_n136, new_n137, new_n138, new_n139,
    new_n140, new_n142, new_n143, new_n144, new_n145, new_n146, new_n147,
    new_n148, new_n149, new_n150, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n159, new_n160, new_n161, new_n162,
    new_n164, new_n165, new_n166, new_n167, new_n168, new_n169, new_n170,
    new_n171, new_n172, new_n174, new_n175, new_n176, new_n177, new_n178,
    new_n179, new_n180, new_n181, new_n183, new_n184, new_n185, new_n186,
    new_n187, new_n188, new_n190, new_n191, new_n192, new_n193, new_n194,
    new_n195, new_n196, new_n197, new_n198, new_n199, new_n201, new_n202,
    new_n203, new_n204, new_n205, new_n206, new_n207, new_n208, new_n210,
    new_n211, new_n212, new_n213, new_n214, new_n215, new_n216, new_n217,
    new_n218, new_n219, new_n220, new_n221, new_n224, new_n225, new_n226,
    new_n227, new_n229, new_n230, new_n231, new_n232, new_n233, new_n234,
    new_n236, new_n237, new_n238, new_n239, new_n240, new_n241, new_n242,
    new_n243, new_n244, new_n246, new_n247, new_n248, new_n249, new_n250,
    new_n251, new_n252, new_n253, new_n254, new_n255, new_n256, new_n258,
    new_n259, new_n260, new_n261, new_n262, new_n263, new_n264, new_n266,
    new_n267, new_n268, new_n269, new_n270, new_n271, new_n272, new_n273,
    new_n274, new_n275, new_n276, new_n277, new_n278, new_n279, new_n280,
    new_n281, new_n282, new_n284, new_n285, new_n286, new_n287, new_n288,
    new_n289, new_n291, new_n292, new_n293, new_n294, new_n295, new_n296,
    new_n297, new_n298, new_n299, new_n300, new_n301, new_n302, new_n304,
    new_n305, new_n306, new_n307, new_n308, new_n309, new_n310, new_n311,
    new_n312, new_n313, new_n315, new_n316, new_n317, new_n318, new_n319,
    new_n320, new_n321, new_n324, new_n325, new_n326, new_n327, new_n328,
    new_n329, new_n330, new_n331, new_n332, new_n333, new_n335, new_n336,
    new_n337, new_n338, new_n339, new_n340, new_n341, new_n343, new_n345,
    new_n348, new_n349, new_n351, new_n352, new_n354, new_n355;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor042aa1d18x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  inv000aa1d42x5               g002(.a(new_n97), .o1(new_n98));
  nor042aa1n06x5               g003(.a(\b[5] ), .b(\a[6] ), .o1(new_n99));
  nand42aa1n02x5               g004(.a(\b[5] ), .b(\a[6] ), .o1(new_n100));
  norb02aa1n09x5               g005(.a(new_n100), .b(new_n99), .out0(new_n101));
  norp02aa1n04x5               g006(.a(\b[4] ), .b(\a[5] ), .o1(new_n102));
  nand42aa1n08x5               g007(.a(\b[4] ), .b(\a[5] ), .o1(new_n103));
  norb02aa1n02x7               g008(.a(new_n103), .b(new_n102), .out0(new_n104));
  nor022aa1n08x5               g009(.a(\b[7] ), .b(\a[8] ), .o1(new_n105));
  nand02aa1d04x5               g010(.a(\b[7] ), .b(\a[8] ), .o1(new_n106));
  nor002aa1d32x5               g011(.a(\b[6] ), .b(\a[7] ), .o1(new_n107));
  nanp02aa1n04x5               g012(.a(\b[6] ), .b(\a[7] ), .o1(new_n108));
  nona23aa1n09x5               g013(.a(new_n108), .b(new_n106), .c(new_n105), .d(new_n107), .out0(new_n109));
  nano22aa1n06x5               g014(.a(new_n109), .b(new_n101), .c(new_n104), .out0(new_n110));
  oa0022aa1n06x5               g015(.a(\a[4] ), .b(\b[3] ), .c(\a[3] ), .d(\b[2] ), .o(new_n111));
  nand22aa1n12x5               g016(.a(\b[0] ), .b(\a[1] ), .o1(new_n112));
  nanp02aa1n04x5               g017(.a(\b[1] ), .b(\a[2] ), .o1(new_n113));
  nor042aa1n06x5               g018(.a(\b[1] ), .b(\a[2] ), .o1(new_n114));
  nand42aa1n02x5               g019(.a(\b[2] ), .b(\a[3] ), .o1(new_n115));
  oai112aa1n06x5               g020(.a(new_n115), .b(new_n113), .c(new_n114), .d(new_n112), .o1(new_n116));
  aoi022aa1d18x5               g021(.a(new_n116), .b(new_n111), .c(\a[4] ), .d(\b[3] ), .o1(new_n117));
  tech160nm_fiaoi012aa1n05x5   g022(.a(new_n99), .b(new_n102), .c(new_n100), .o1(new_n118));
  inv040aa1n02x5               g023(.a(new_n107), .o1(new_n119));
  oaoi03aa1n02x5               g024(.a(\a[8] ), .b(\b[7] ), .c(new_n119), .o1(new_n120));
  oabi12aa1n06x5               g025(.a(new_n120), .b(new_n109), .c(new_n118), .out0(new_n121));
  nand42aa1n04x5               g026(.a(\b[8] ), .b(\a[9] ), .o1(new_n122));
  norb02aa1n02x5               g027(.a(new_n122), .b(new_n97), .out0(new_n123));
  aoai13aa1n06x5               g028(.a(new_n123), .b(new_n121), .c(new_n117), .d(new_n110), .o1(new_n124));
  nor042aa1d18x5               g029(.a(\b[9] ), .b(\a[10] ), .o1(new_n125));
  nanp02aa1n04x5               g030(.a(\b[9] ), .b(\a[10] ), .o1(new_n126));
  norb02aa1n09x5               g031(.a(new_n126), .b(new_n125), .out0(new_n127));
  xnbna2aa1n03x5               g032(.a(new_n127), .b(new_n124), .c(new_n98), .out0(\s[10] ));
  inv000aa1d42x5               g033(.a(new_n125), .o1(new_n129));
  nano23aa1n06x5               g034(.a(new_n105), .b(new_n107), .c(new_n108), .d(new_n106), .out0(new_n130));
  nanp03aa1n02x5               g035(.a(new_n130), .b(new_n101), .c(new_n104), .o1(new_n131));
  nanp02aa1n02x5               g036(.a(\b[3] ), .b(\a[4] ), .o1(new_n132));
  nand42aa1n03x5               g037(.a(new_n116), .b(new_n111), .o1(new_n133));
  nanp02aa1n02x5               g038(.a(new_n133), .b(new_n132), .o1(new_n134));
  aoib12aa1n02x5               g039(.a(new_n120), .b(new_n130), .c(new_n118), .out0(new_n135));
  tech160nm_fioai012aa1n04x5   g040(.a(new_n135), .b(new_n134), .c(new_n131), .o1(new_n136));
  aoai13aa1n02x5               g041(.a(new_n127), .b(new_n97), .c(new_n136), .d(new_n122), .o1(new_n137));
  nor042aa1n09x5               g042(.a(\b[10] ), .b(\a[11] ), .o1(new_n138));
  nanp02aa1n09x5               g043(.a(\b[10] ), .b(\a[11] ), .o1(new_n139));
  norb02aa1n02x5               g044(.a(new_n139), .b(new_n138), .out0(new_n140));
  xnbna2aa1n03x5               g045(.a(new_n140), .b(new_n137), .c(new_n129), .out0(\s[11] ));
  inv000aa1d42x5               g046(.a(new_n127), .o1(new_n142));
  aoai13aa1n04x5               g047(.a(new_n129), .b(new_n142), .c(new_n124), .d(new_n98), .o1(new_n143));
  nor042aa1n06x5               g048(.a(\b[11] ), .b(\a[12] ), .o1(new_n144));
  nand22aa1n09x5               g049(.a(\b[11] ), .b(\a[12] ), .o1(new_n145));
  norb02aa1n02x5               g050(.a(new_n145), .b(new_n144), .out0(new_n146));
  inv000aa1d42x5               g051(.a(new_n146), .o1(new_n147));
  aoai13aa1n02x5               g052(.a(new_n147), .b(new_n138), .c(new_n143), .d(new_n140), .o1(new_n148));
  nanp02aa1n02x5               g053(.a(new_n143), .b(new_n140), .o1(new_n149));
  nona22aa1n02x4               g054(.a(new_n149), .b(new_n147), .c(new_n138), .out0(new_n150));
  nanp02aa1n02x5               g055(.a(new_n150), .b(new_n148), .o1(\s[12] ));
  nano23aa1d15x5               g056(.a(new_n138), .b(new_n144), .c(new_n145), .d(new_n139), .out0(new_n152));
  nano23aa1d15x5               g057(.a(new_n97), .b(new_n125), .c(new_n126), .d(new_n122), .out0(new_n153));
  nand22aa1n12x5               g058(.a(new_n153), .b(new_n152), .o1(new_n154));
  inv000aa1d42x5               g059(.a(new_n154), .o1(new_n155));
  aoai13aa1n06x5               g060(.a(new_n155), .b(new_n121), .c(new_n117), .d(new_n110), .o1(new_n156));
  tech160nm_fioai012aa1n05x5   g061(.a(new_n126), .b(new_n125), .c(new_n97), .o1(new_n157));
  ao0012aa1n03x7               g062(.a(new_n144), .b(new_n138), .c(new_n145), .o(new_n158));
  aoib12aa1n06x5               g063(.a(new_n158), .b(new_n152), .c(new_n157), .out0(new_n159));
  norp02aa1n04x5               g064(.a(\b[12] ), .b(\a[13] ), .o1(new_n160));
  nand42aa1n06x5               g065(.a(\b[12] ), .b(\a[13] ), .o1(new_n161));
  norb02aa1n03x5               g066(.a(new_n161), .b(new_n160), .out0(new_n162));
  xnbna2aa1n03x5               g067(.a(new_n162), .b(new_n156), .c(new_n159), .out0(\s[13] ));
  inv000aa1d42x5               g068(.a(\a[13] ), .o1(new_n164));
  inv000aa1d42x5               g069(.a(\b[12] ), .o1(new_n165));
  nanp02aa1n02x5               g070(.a(new_n165), .b(new_n164), .o1(new_n166));
  nona23aa1n09x5               g071(.a(new_n145), .b(new_n139), .c(new_n138), .d(new_n144), .out0(new_n167));
  oabi12aa1n18x5               g072(.a(new_n158), .b(new_n167), .c(new_n157), .out0(new_n168));
  aoai13aa1n02x5               g073(.a(new_n162), .b(new_n168), .c(new_n136), .d(new_n155), .o1(new_n169));
  norp02aa1n12x5               g074(.a(\b[13] ), .b(\a[14] ), .o1(new_n170));
  nand42aa1n06x5               g075(.a(\b[13] ), .b(\a[14] ), .o1(new_n171));
  norb02aa1n03x5               g076(.a(new_n171), .b(new_n170), .out0(new_n172));
  xnbna2aa1n03x5               g077(.a(new_n172), .b(new_n169), .c(new_n166), .out0(\s[14] ));
  nona23aa1n09x5               g078(.a(new_n171), .b(new_n161), .c(new_n160), .d(new_n170), .out0(new_n174));
  aoi012aa1n02x5               g079(.a(new_n174), .b(new_n156), .c(new_n159), .o1(new_n175));
  aoai13aa1n04x5               g080(.a(new_n171), .b(new_n170), .c(new_n164), .d(new_n165), .o1(new_n176));
  aoai13aa1n06x5               g081(.a(new_n176), .b(new_n174), .c(new_n156), .d(new_n159), .o1(new_n177));
  nor042aa1n04x5               g082(.a(\b[14] ), .b(\a[15] ), .o1(new_n178));
  nand42aa1d28x5               g083(.a(\b[14] ), .b(\a[15] ), .o1(new_n179));
  norb02aa1n06x5               g084(.a(new_n179), .b(new_n178), .out0(new_n180));
  oaoi13aa1n02x5               g085(.a(new_n180), .b(new_n171), .c(new_n160), .d(new_n170), .o1(new_n181));
  aboi22aa1n03x5               g086(.a(new_n175), .b(new_n181), .c(new_n177), .d(new_n180), .out0(\s[15] ));
  nor022aa1n04x5               g087(.a(\b[15] ), .b(\a[16] ), .o1(new_n183));
  nand42aa1n08x5               g088(.a(\b[15] ), .b(\a[16] ), .o1(new_n184));
  nanb02aa1n02x5               g089(.a(new_n183), .b(new_n184), .out0(new_n185));
  aoai13aa1n03x5               g090(.a(new_n185), .b(new_n178), .c(new_n177), .d(new_n179), .o1(new_n186));
  nanp02aa1n02x5               g091(.a(new_n177), .b(new_n180), .o1(new_n187));
  nona22aa1n02x4               g092(.a(new_n187), .b(new_n185), .c(new_n178), .out0(new_n188));
  nanp02aa1n02x5               g093(.a(new_n188), .b(new_n186), .o1(\s[16] ));
  nano23aa1n06x5               g094(.a(new_n178), .b(new_n183), .c(new_n184), .d(new_n179), .out0(new_n190));
  nano32aa1d12x5               g095(.a(new_n154), .b(new_n190), .c(new_n162), .d(new_n172), .out0(new_n191));
  aoai13aa1n06x5               g096(.a(new_n191), .b(new_n121), .c(new_n110), .d(new_n117), .o1(new_n192));
  inv040aa1n02x5               g097(.a(new_n183), .o1(new_n193));
  nano32aa1n03x7               g098(.a(new_n174), .b(new_n184), .c(new_n180), .d(new_n193), .out0(new_n194));
  inv000aa1n02x5               g099(.a(new_n178), .o1(new_n195));
  inv000aa1d42x5               g100(.a(new_n179), .o1(new_n196));
  aoai13aa1n12x5               g101(.a(new_n193), .b(new_n196), .c(new_n176), .d(new_n195), .o1(new_n197));
  aoi022aa1d18x5               g102(.a(new_n168), .b(new_n194), .c(new_n197), .d(new_n184), .o1(new_n198));
  tech160nm_fixnrc02aa1n04x5   g103(.a(\b[16] ), .b(\a[17] ), .out0(new_n199));
  xobna2aa1n03x5               g104(.a(new_n199), .b(new_n192), .c(new_n198), .out0(\s[17] ));
  norp02aa1n02x5               g105(.a(\b[16] ), .b(\a[17] ), .o1(new_n201));
  inv000aa1d42x5               g106(.a(new_n201), .o1(new_n202));
  inv000aa1d42x5               g107(.a(\a[17] ), .o1(new_n203));
  aoi012aa1n12x5               g108(.a(new_n121), .b(new_n117), .c(new_n110), .o1(new_n204));
  nona23aa1n09x5               g109(.a(new_n190), .b(new_n153), .c(new_n167), .d(new_n174), .out0(new_n205));
  oai012aa1d24x5               g110(.a(new_n198), .b(new_n204), .c(new_n205), .o1(new_n206));
  oaib12aa1n06x5               g111(.a(new_n206), .b(new_n203), .c(\b[16] ), .out0(new_n207));
  tech160nm_fixorc02aa1n03p5x5 g112(.a(\a[18] ), .b(\b[17] ), .out0(new_n208));
  xnbna2aa1n03x5               g113(.a(new_n208), .b(new_n207), .c(new_n202), .out0(\s[18] ));
  inv000aa1d42x5               g114(.a(\a[18] ), .o1(new_n210));
  xroi22aa1d06x4               g115(.a(new_n203), .b(\b[16] ), .c(new_n210), .d(\b[17] ), .out0(new_n211));
  inv000aa1d42x5               g116(.a(new_n211), .o1(new_n212));
  nand02aa1d06x5               g117(.a(\b[17] ), .b(\a[18] ), .o1(new_n213));
  oai022aa1d24x5               g118(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n214));
  nanp02aa1n02x5               g119(.a(new_n214), .b(new_n213), .o1(new_n215));
  aoai13aa1n04x5               g120(.a(new_n215), .b(new_n212), .c(new_n192), .d(new_n198), .o1(new_n216));
  nor002aa1n06x5               g121(.a(\b[18] ), .b(\a[19] ), .o1(new_n217));
  tech160nm_finand02aa1n03p5x5 g122(.a(\b[18] ), .b(\a[19] ), .o1(new_n218));
  norb02aa1n06x5               g123(.a(new_n218), .b(new_n217), .out0(new_n219));
  inv000aa1d42x5               g124(.a(new_n215), .o1(new_n220));
  aoi112aa1n02x5               g125(.a(new_n219), .b(new_n220), .c(new_n206), .d(new_n211), .o1(new_n221));
  aoi012aa1n02x5               g126(.a(new_n221), .b(new_n216), .c(new_n219), .o1(\s[19] ));
  xnrc02aa1n02x5               g127(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  tech160nm_fixnrc02aa1n04x5   g128(.a(\b[19] ), .b(\a[20] ), .out0(new_n224));
  aoai13aa1n03x5               g129(.a(new_n224), .b(new_n217), .c(new_n216), .d(new_n218), .o1(new_n225));
  aoai13aa1n03x5               g130(.a(new_n219), .b(new_n220), .c(new_n206), .d(new_n211), .o1(new_n226));
  nona22aa1n03x5               g131(.a(new_n226), .b(new_n224), .c(new_n217), .out0(new_n227));
  nanp02aa1n03x5               g132(.a(new_n225), .b(new_n227), .o1(\s[20] ));
  nano23aa1n09x5               g133(.a(new_n224), .b(new_n199), .c(new_n208), .d(new_n219), .out0(new_n229));
  inv020aa1n08x5               g134(.a(new_n229), .o1(new_n230));
  aoai13aa1n09x5               g135(.a(new_n218), .b(new_n217), .c(new_n214), .d(new_n213), .o1(new_n231));
  oaoi03aa1n12x5               g136(.a(\a[20] ), .b(\b[19] ), .c(new_n231), .o1(new_n232));
  inv000aa1d42x5               g137(.a(new_n232), .o1(new_n233));
  aoai13aa1n04x5               g138(.a(new_n233), .b(new_n230), .c(new_n192), .d(new_n198), .o1(new_n234));
  xorb03aa1n02x5               g139(.a(new_n234), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor042aa1n04x5               g140(.a(\b[20] ), .b(\a[21] ), .o1(new_n236));
  nand42aa1n08x5               g141(.a(\b[20] ), .b(\a[21] ), .o1(new_n237));
  norb02aa1n02x5               g142(.a(new_n237), .b(new_n236), .out0(new_n238));
  norp02aa1n09x5               g143(.a(\b[21] ), .b(\a[22] ), .o1(new_n239));
  nand42aa1n08x5               g144(.a(\b[21] ), .b(\a[22] ), .o1(new_n240));
  nanb02aa1n02x5               g145(.a(new_n239), .b(new_n240), .out0(new_n241));
  aoai13aa1n03x5               g146(.a(new_n241), .b(new_n236), .c(new_n234), .d(new_n238), .o1(new_n242));
  aoai13aa1n06x5               g147(.a(new_n238), .b(new_n232), .c(new_n206), .d(new_n229), .o1(new_n243));
  nona22aa1n03x5               g148(.a(new_n243), .b(new_n241), .c(new_n236), .out0(new_n244));
  nanp02aa1n03x5               g149(.a(new_n242), .b(new_n244), .o1(\s[22] ));
  nano23aa1d18x5               g150(.a(new_n236), .b(new_n239), .c(new_n240), .d(new_n237), .out0(new_n246));
  nano32aa1n03x7               g151(.a(new_n224), .b(new_n211), .c(new_n246), .d(new_n219), .out0(new_n247));
  inv020aa1n03x5               g152(.a(new_n247), .o1(new_n248));
  tech160nm_fioai012aa1n04x5   g153(.a(new_n240), .b(new_n239), .c(new_n236), .o1(new_n249));
  aobi12aa1n06x5               g154(.a(new_n249), .b(new_n232), .c(new_n246), .out0(new_n250));
  aoai13aa1n04x5               g155(.a(new_n250), .b(new_n248), .c(new_n192), .d(new_n198), .o1(new_n251));
  nor042aa1n04x5               g156(.a(\b[22] ), .b(\a[23] ), .o1(new_n252));
  nand42aa1n06x5               g157(.a(\b[22] ), .b(\a[23] ), .o1(new_n253));
  norb02aa1n02x5               g158(.a(new_n253), .b(new_n252), .out0(new_n254));
  inv040aa1n03x5               g159(.a(new_n250), .o1(new_n255));
  aoi112aa1n02x5               g160(.a(new_n254), .b(new_n255), .c(new_n206), .d(new_n247), .o1(new_n256));
  aoi012aa1n02x5               g161(.a(new_n256), .b(new_n251), .c(new_n254), .o1(\s[23] ));
  nor042aa1n03x5               g162(.a(\b[23] ), .b(\a[24] ), .o1(new_n258));
  nand42aa1n06x5               g163(.a(\b[23] ), .b(\a[24] ), .o1(new_n259));
  norb02aa1n02x5               g164(.a(new_n259), .b(new_n258), .out0(new_n260));
  inv000aa1d42x5               g165(.a(new_n260), .o1(new_n261));
  aoai13aa1n03x5               g166(.a(new_n261), .b(new_n252), .c(new_n251), .d(new_n254), .o1(new_n262));
  aoai13aa1n06x5               g167(.a(new_n254), .b(new_n255), .c(new_n206), .d(new_n247), .o1(new_n263));
  nona22aa1n03x5               g168(.a(new_n263), .b(new_n261), .c(new_n252), .out0(new_n264));
  nanp02aa1n03x5               g169(.a(new_n262), .b(new_n264), .o1(\s[24] ));
  nanp03aa1n02x5               g170(.a(new_n190), .b(new_n162), .c(new_n172), .o1(new_n266));
  nanp02aa1n02x5               g171(.a(new_n197), .b(new_n184), .o1(new_n267));
  tech160nm_fioai012aa1n03p5x5 g172(.a(new_n267), .b(new_n159), .c(new_n266), .o1(new_n268));
  inv000aa1d42x5               g173(.a(new_n246), .o1(new_n269));
  nano23aa1d12x5               g174(.a(new_n252), .b(new_n258), .c(new_n259), .d(new_n253), .out0(new_n270));
  inv000aa1d42x5               g175(.a(new_n270), .o1(new_n271));
  nona22aa1n12x5               g176(.a(new_n229), .b(new_n269), .c(new_n271), .out0(new_n272));
  inv000aa1n02x5               g177(.a(new_n272), .o1(new_n273));
  aoai13aa1n02x5               g178(.a(new_n273), .b(new_n268), .c(new_n136), .d(new_n191), .o1(new_n274));
  nand02aa1d04x5               g179(.a(new_n270), .b(new_n246), .o1(new_n275));
  inv000aa1n09x5               g180(.a(new_n275), .o1(new_n276));
  tech160nm_fioai012aa1n03p5x5 g181(.a(new_n259), .b(new_n258), .c(new_n252), .o1(new_n277));
  oaib12aa1n18x5               g182(.a(new_n277), .b(new_n249), .c(new_n270), .out0(new_n278));
  tech160nm_fiaoi012aa1n05x5   g183(.a(new_n278), .b(new_n232), .c(new_n276), .o1(new_n279));
  aoai13aa1n09x5               g184(.a(new_n279), .b(new_n272), .c(new_n192), .d(new_n198), .o1(new_n280));
  tech160nm_fixorc02aa1n03p5x5 g185(.a(\a[25] ), .b(\b[24] ), .out0(new_n281));
  aoi112aa1n02x5               g186(.a(new_n278), .b(new_n281), .c(new_n232), .d(new_n276), .o1(new_n282));
  aoi022aa1n02x5               g187(.a(new_n280), .b(new_n281), .c(new_n274), .d(new_n282), .o1(\s[25] ));
  norp02aa1n02x5               g188(.a(\b[24] ), .b(\a[25] ), .o1(new_n284));
  tech160nm_fixnrc02aa1n05x5   g189(.a(\b[25] ), .b(\a[26] ), .out0(new_n285));
  aoai13aa1n03x5               g190(.a(new_n285), .b(new_n284), .c(new_n280), .d(new_n281), .o1(new_n286));
  inv040aa1n02x5               g191(.a(new_n279), .o1(new_n287));
  aoai13aa1n06x5               g192(.a(new_n281), .b(new_n287), .c(new_n206), .d(new_n273), .o1(new_n288));
  nona22aa1n03x5               g193(.a(new_n288), .b(new_n285), .c(new_n284), .out0(new_n289));
  nanp02aa1n03x5               g194(.a(new_n286), .b(new_n289), .o1(\s[26] ));
  norb02aa1n02x7               g195(.a(new_n281), .b(new_n285), .out0(new_n291));
  nano32aa1n03x7               g196(.a(new_n230), .b(new_n291), .c(new_n246), .d(new_n270), .out0(new_n292));
  aoai13aa1n06x5               g197(.a(new_n292), .b(new_n268), .c(new_n136), .d(new_n191), .o1(new_n293));
  orn002aa1n02x5               g198(.a(\a[20] ), .b(\b[19] ), .o(new_n294));
  and002aa1n02x5               g199(.a(\b[19] ), .b(\a[20] ), .o(new_n295));
  aoi112aa1n03x4               g200(.a(new_n275), .b(new_n295), .c(new_n231), .d(new_n294), .o1(new_n296));
  inv000aa1d42x5               g201(.a(\a[26] ), .o1(new_n297));
  inv000aa1d42x5               g202(.a(\b[25] ), .o1(new_n298));
  oaoi03aa1n02x5               g203(.a(new_n297), .b(new_n298), .c(new_n284), .o1(new_n299));
  inv000aa1n02x5               g204(.a(new_n299), .o1(new_n300));
  oaoi13aa1n06x5               g205(.a(new_n300), .b(new_n291), .c(new_n296), .d(new_n278), .o1(new_n301));
  xnrc02aa1n12x5               g206(.a(\b[26] ), .b(\a[27] ), .out0(new_n302));
  xobna2aa1n06x5               g207(.a(new_n302), .b(new_n293), .c(new_n301), .out0(\s[27] ));
  aoai13aa1n06x5               g208(.a(new_n291), .b(new_n278), .c(new_n232), .d(new_n276), .o1(new_n304));
  tech160nm_finand02aa1n03p5x5 g209(.a(new_n304), .b(new_n299), .o1(new_n305));
  and002aa1n02x5               g210(.a(\b[26] ), .b(\a[27] ), .o(new_n306));
  inv000aa1d42x5               g211(.a(new_n306), .o1(new_n307));
  aoai13aa1n03x5               g212(.a(new_n307), .b(new_n305), .c(new_n206), .d(new_n292), .o1(new_n308));
  norp02aa1n02x5               g213(.a(\b[26] ), .b(\a[27] ), .o1(new_n309));
  inv000aa1n03x5               g214(.a(new_n309), .o1(new_n310));
  aoai13aa1n02x7               g215(.a(new_n310), .b(new_n306), .c(new_n293), .d(new_n301), .o1(new_n311));
  xorc02aa1n02x5               g216(.a(\a[28] ), .b(\b[27] ), .out0(new_n312));
  norp02aa1n02x5               g217(.a(new_n312), .b(new_n309), .o1(new_n313));
  aoi022aa1n03x5               g218(.a(new_n311), .b(new_n312), .c(new_n308), .d(new_n313), .o1(\s[28] ));
  norb02aa1n02x5               g219(.a(new_n312), .b(new_n302), .out0(new_n315));
  aoai13aa1n03x5               g220(.a(new_n315), .b(new_n305), .c(new_n206), .d(new_n292), .o1(new_n316));
  inv000aa1n02x5               g221(.a(new_n315), .o1(new_n317));
  oao003aa1n02x5               g222(.a(\a[28] ), .b(\b[27] ), .c(new_n310), .carry(new_n318));
  aoai13aa1n03x5               g223(.a(new_n318), .b(new_n317), .c(new_n293), .d(new_n301), .o1(new_n319));
  xorc02aa1n02x5               g224(.a(\a[29] ), .b(\b[28] ), .out0(new_n320));
  norb02aa1n02x5               g225(.a(new_n318), .b(new_n320), .out0(new_n321));
  aoi022aa1n03x5               g226(.a(new_n319), .b(new_n320), .c(new_n316), .d(new_n321), .o1(\s[29] ));
  xorb03aa1n02x5               g227(.a(new_n112), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n06x5               g228(.a(new_n302), .b(new_n312), .c(new_n320), .out0(new_n324));
  aoai13aa1n03x5               g229(.a(new_n324), .b(new_n305), .c(new_n206), .d(new_n292), .o1(new_n325));
  inv000aa1d42x5               g230(.a(new_n324), .o1(new_n326));
  oaoi03aa1n02x5               g231(.a(\a[29] ), .b(\b[28] ), .c(new_n318), .o1(new_n327));
  inv000aa1n03x5               g232(.a(new_n327), .o1(new_n328));
  aoai13aa1n03x5               g233(.a(new_n328), .b(new_n326), .c(new_n293), .d(new_n301), .o1(new_n329));
  xorc02aa1n02x5               g234(.a(\a[30] ), .b(\b[29] ), .out0(new_n330));
  and002aa1n02x5               g235(.a(\b[28] ), .b(\a[29] ), .o(new_n331));
  oabi12aa1n02x5               g236(.a(new_n330), .b(\a[29] ), .c(\b[28] ), .out0(new_n332));
  oab012aa1n02x4               g237(.a(new_n332), .b(new_n318), .c(new_n331), .out0(new_n333));
  aoi022aa1n03x5               g238(.a(new_n329), .b(new_n330), .c(new_n325), .d(new_n333), .o1(\s[30] ));
  nano32aa1n06x5               g239(.a(new_n302), .b(new_n330), .c(new_n312), .d(new_n320), .out0(new_n335));
  aoai13aa1n03x5               g240(.a(new_n335), .b(new_n305), .c(new_n206), .d(new_n292), .o1(new_n336));
  xorc02aa1n02x5               g241(.a(\a[31] ), .b(\b[30] ), .out0(new_n337));
  oao003aa1n02x5               g242(.a(\a[30] ), .b(\b[29] ), .c(new_n328), .carry(new_n338));
  norb02aa1n02x5               g243(.a(new_n338), .b(new_n337), .out0(new_n339));
  inv000aa1d42x5               g244(.a(new_n335), .o1(new_n340));
  aoai13aa1n03x5               g245(.a(new_n338), .b(new_n340), .c(new_n293), .d(new_n301), .o1(new_n341));
  aoi022aa1n03x5               g246(.a(new_n341), .b(new_n337), .c(new_n336), .d(new_n339), .o1(\s[31] ));
  oai012aa1n02x5               g247(.a(new_n113), .b(new_n114), .c(new_n112), .o1(new_n343));
  xnrb03aa1n02x5               g248(.a(new_n343), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g249(.a(\a[3] ), .b(\b[2] ), .c(new_n343), .o1(new_n345));
  xorb03aa1n02x5               g250(.a(new_n345), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g251(.a(new_n117), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  oaoi13aa1n02x5               g252(.a(new_n101), .b(new_n103), .c(new_n117), .d(new_n102), .o1(new_n348));
  oai112aa1n02x5               g253(.a(new_n101), .b(new_n103), .c(new_n117), .d(new_n102), .o1(new_n349));
  norb02aa1n02x5               g254(.a(new_n349), .b(new_n348), .out0(\s[6] ));
  inv000aa1d42x5               g255(.a(new_n99), .o1(new_n351));
  norb02aa1n02x5               g256(.a(new_n108), .b(new_n107), .out0(new_n352));
  xnbna2aa1n03x5               g257(.a(new_n352), .b(new_n349), .c(new_n351), .out0(\s[7] ));
  inv000aa1n02x5               g258(.a(new_n352), .o1(new_n354));
  aoai13aa1n03x5               g259(.a(new_n119), .b(new_n354), .c(new_n349), .d(new_n351), .o1(new_n355));
  xorb03aa1n02x5               g260(.a(new_n355), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xnbna2aa1n03x5               g261(.a(new_n204), .b(new_n122), .c(new_n98), .out0(\s[9] ));
endmodule


