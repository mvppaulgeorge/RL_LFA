// Benchmark "adder" written by ABC on Thu Jul 18 08:39:10 2024

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
    new_n140, new_n141, new_n142, new_n143, new_n144, new_n146, new_n147,
    new_n148, new_n149, new_n150, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n159, new_n161, new_n162, new_n164,
    new_n165, new_n166, new_n167, new_n168, new_n169, new_n170, new_n171,
    new_n173, new_n174, new_n175, new_n176, new_n177, new_n178, new_n179,
    new_n180, new_n182, new_n183, new_n184, new_n185, new_n186, new_n187,
    new_n188, new_n189, new_n190, new_n191, new_n192, new_n193, new_n194,
    new_n195, new_n196, new_n197, new_n198, new_n199, new_n200, new_n201,
    new_n202, new_n204, new_n205, new_n206, new_n207, new_n208, new_n209,
    new_n210, new_n211, new_n212, new_n213, new_n215, new_n216, new_n217,
    new_n218, new_n219, new_n222, new_n223, new_n224, new_n225, new_n227,
    new_n228, new_n229, new_n230, new_n231, new_n232, new_n233, new_n234,
    new_n235, new_n236, new_n237, new_n238, new_n239, new_n240, new_n242,
    new_n243, new_n244, new_n245, new_n246, new_n247, new_n249, new_n250,
    new_n251, new_n252, new_n253, new_n254, new_n255, new_n256, new_n257,
    new_n258, new_n259, new_n260, new_n262, new_n263, new_n264, new_n265,
    new_n267, new_n268, new_n269, new_n270, new_n271, new_n272, new_n273,
    new_n274, new_n275, new_n276, new_n277, new_n278, new_n279, new_n280,
    new_n282, new_n283, new_n284, new_n285, new_n287, new_n288, new_n289,
    new_n290, new_n291, new_n292, new_n293, new_n294, new_n295, new_n296,
    new_n297, new_n298, new_n300, new_n301, new_n302, new_n303, new_n304,
    new_n306, new_n307, new_n308, new_n309, new_n310, new_n311, new_n312,
    new_n313, new_n314, new_n317, new_n318, new_n319, new_n320, new_n321,
    new_n322, new_n323, new_n324, new_n325, new_n327, new_n328, new_n329,
    new_n330, new_n331, new_n332, new_n333, new_n334, new_n335, new_n337,
    new_n339, new_n340, new_n343, new_n345, new_n346, new_n347, new_n349,
    new_n350, new_n351, new_n352;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor042aa1n02x5               g001(.a(\b[1] ), .b(\a[2] ), .o1(new_n97));
  inv000aa1n02x5               g002(.a(new_n97), .o1(new_n98));
  nand02aa1n06x5               g003(.a(\b[0] ), .b(\a[1] ), .o1(new_n99));
  nand42aa1n04x5               g004(.a(\b[1] ), .b(\a[2] ), .o1(new_n100));
  aob012aa1n02x5               g005(.a(new_n98), .b(new_n99), .c(new_n100), .out0(new_n101));
  nor042aa1n06x5               g006(.a(\b[3] ), .b(\a[4] ), .o1(new_n102));
  nand22aa1n02x5               g007(.a(\b[3] ), .b(\a[4] ), .o1(new_n103));
  norb02aa1n03x5               g008(.a(new_n103), .b(new_n102), .out0(new_n104));
  nor002aa1n02x5               g009(.a(\b[2] ), .b(\a[3] ), .o1(new_n105));
  nand02aa1n03x5               g010(.a(\b[2] ), .b(\a[3] ), .o1(new_n106));
  norb02aa1n06x4               g011(.a(new_n106), .b(new_n105), .out0(new_n107));
  nanp03aa1n03x5               g012(.a(new_n101), .b(new_n104), .c(new_n107), .o1(new_n108));
  inv040aa1d32x5               g013(.a(\a[3] ), .o1(new_n109));
  inv000aa1d42x5               g014(.a(\b[2] ), .o1(new_n110));
  nanp02aa1n12x5               g015(.a(new_n110), .b(new_n109), .o1(new_n111));
  tech160nm_fioaoi03aa1n05x5   g016(.a(\a[4] ), .b(\b[3] ), .c(new_n111), .o1(new_n112));
  inv000aa1n02x5               g017(.a(new_n112), .o1(new_n113));
  norp02aa1n12x5               g018(.a(\b[7] ), .b(\a[8] ), .o1(new_n114));
  nand02aa1d28x5               g019(.a(\b[7] ), .b(\a[8] ), .o1(new_n115));
  nor022aa1n16x5               g020(.a(\b[6] ), .b(\a[7] ), .o1(new_n116));
  nand02aa1n12x5               g021(.a(\b[6] ), .b(\a[7] ), .o1(new_n117));
  nano23aa1n03x7               g022(.a(new_n114), .b(new_n116), .c(new_n117), .d(new_n115), .out0(new_n118));
  tech160nm_fixorc02aa1n02p5x5 g023(.a(\a[6] ), .b(\b[5] ), .out0(new_n119));
  tech160nm_fixorc02aa1n02p5x5 g024(.a(\a[5] ), .b(\b[4] ), .out0(new_n120));
  nand23aa1n03x5               g025(.a(new_n118), .b(new_n119), .c(new_n120), .o1(new_n121));
  nor042aa1d18x5               g026(.a(\b[4] ), .b(\a[5] ), .o1(new_n122));
  inv000aa1n03x5               g027(.a(new_n122), .o1(new_n123));
  oaoi03aa1n02x5               g028(.a(\a[6] ), .b(\b[5] ), .c(new_n123), .o1(new_n124));
  tech160nm_fiao0012aa1n02p5x5 g029(.a(new_n114), .b(new_n116), .c(new_n115), .o(new_n125));
  aoi012aa1n09x5               g030(.a(new_n125), .b(new_n118), .c(new_n124), .o1(new_n126));
  aoai13aa1n12x5               g031(.a(new_n126), .b(new_n121), .c(new_n108), .d(new_n113), .o1(new_n127));
  nor002aa1n16x5               g032(.a(\b[8] ), .b(\a[9] ), .o1(new_n128));
  nand02aa1n06x5               g033(.a(\b[8] ), .b(\a[9] ), .o1(new_n129));
  norb02aa1n06x4               g034(.a(new_n129), .b(new_n128), .out0(new_n130));
  nor042aa1n06x5               g035(.a(\b[9] ), .b(\a[10] ), .o1(new_n131));
  nand02aa1d28x5               g036(.a(\b[9] ), .b(\a[10] ), .o1(new_n132));
  norb02aa1n06x4               g037(.a(new_n132), .b(new_n131), .out0(new_n133));
  aoi112aa1n02x5               g038(.a(new_n128), .b(new_n133), .c(new_n127), .d(new_n130), .o1(new_n134));
  aoai13aa1n02x5               g039(.a(new_n133), .b(new_n128), .c(new_n127), .d(new_n129), .o1(new_n135));
  norb02aa1n02x5               g040(.a(new_n135), .b(new_n134), .out0(\s[10] ));
  nano23aa1n02x4               g041(.a(new_n128), .b(new_n131), .c(new_n132), .d(new_n129), .out0(new_n137));
  aoi012aa1d24x5               g042(.a(new_n131), .b(new_n128), .c(new_n132), .o1(new_n138));
  inv000aa1d42x5               g043(.a(new_n138), .o1(new_n139));
  nor022aa1n16x5               g044(.a(\b[10] ), .b(\a[11] ), .o1(new_n140));
  nanp02aa1n04x5               g045(.a(\b[10] ), .b(\a[11] ), .o1(new_n141));
  norb02aa1n02x5               g046(.a(new_n141), .b(new_n140), .out0(new_n142));
  aoai13aa1n03x5               g047(.a(new_n142), .b(new_n139), .c(new_n127), .d(new_n137), .o1(new_n143));
  aoi112aa1n02x5               g048(.a(new_n142), .b(new_n139), .c(new_n127), .d(new_n137), .o1(new_n144));
  norb02aa1n02x5               g049(.a(new_n143), .b(new_n144), .out0(\s[11] ));
  nor022aa1n16x5               g050(.a(\b[11] ), .b(\a[12] ), .o1(new_n146));
  nand02aa1d06x5               g051(.a(\b[11] ), .b(\a[12] ), .o1(new_n147));
  norb02aa1n02x5               g052(.a(new_n147), .b(new_n146), .out0(new_n148));
  aoib12aa1n02x5               g053(.a(new_n140), .b(new_n147), .c(new_n146), .out0(new_n149));
  oai012aa1n02x5               g054(.a(new_n143), .b(\b[10] ), .c(\a[11] ), .o1(new_n150));
  aoi022aa1n02x5               g055(.a(new_n150), .b(new_n148), .c(new_n143), .d(new_n149), .o1(\s[12] ));
  nona23aa1n09x5               g056(.a(new_n147), .b(new_n141), .c(new_n140), .d(new_n146), .out0(new_n152));
  nano22aa1n03x7               g057(.a(new_n152), .b(new_n130), .c(new_n133), .out0(new_n153));
  tech160nm_fiaoi012aa1n03p5x5 g058(.a(new_n146), .b(new_n140), .c(new_n147), .o1(new_n154));
  oai012aa1n18x5               g059(.a(new_n154), .b(new_n152), .c(new_n138), .o1(new_n155));
  xnrc02aa1n12x5               g060(.a(\b[12] ), .b(\a[13] ), .out0(new_n156));
  inv000aa1d42x5               g061(.a(new_n156), .o1(new_n157));
  aoai13aa1n03x5               g062(.a(new_n157), .b(new_n155), .c(new_n127), .d(new_n153), .o1(new_n158));
  aoi112aa1n02x5               g063(.a(new_n157), .b(new_n155), .c(new_n127), .d(new_n153), .o1(new_n159));
  norb02aa1n02x5               g064(.a(new_n158), .b(new_n159), .out0(\s[13] ));
  orn002aa1n02x5               g065(.a(\a[13] ), .b(\b[12] ), .o(new_n161));
  xnrc02aa1n06x5               g066(.a(\b[13] ), .b(\a[14] ), .out0(new_n162));
  xobna2aa1n03x5               g067(.a(new_n162), .b(new_n158), .c(new_n161), .out0(\s[14] ));
  nor002aa1n02x5               g068(.a(new_n162), .b(new_n156), .o1(new_n164));
  aoai13aa1n06x5               g069(.a(new_n164), .b(new_n155), .c(new_n127), .d(new_n153), .o1(new_n165));
  norp02aa1n02x5               g070(.a(\b[13] ), .b(\a[14] ), .o1(new_n166));
  aoi112aa1n06x5               g071(.a(\b[12] ), .b(\a[13] ), .c(\a[14] ), .d(\b[13] ), .o1(new_n167));
  nor002aa1n03x5               g072(.a(new_n167), .b(new_n166), .o1(new_n168));
  nor002aa1d32x5               g073(.a(\b[14] ), .b(\a[15] ), .o1(new_n169));
  nanp02aa1n04x5               g074(.a(\b[14] ), .b(\a[15] ), .o1(new_n170));
  norb02aa1n09x5               g075(.a(new_n170), .b(new_n169), .out0(new_n171));
  xnbna2aa1n03x5               g076(.a(new_n171), .b(new_n165), .c(new_n168), .out0(\s[15] ));
  aob012aa1n03x5               g077(.a(new_n171), .b(new_n165), .c(new_n168), .out0(new_n173));
  nor022aa1n16x5               g078(.a(\b[15] ), .b(\a[16] ), .o1(new_n174));
  nand02aa1n04x5               g079(.a(\b[15] ), .b(\a[16] ), .o1(new_n175));
  norb02aa1n02x5               g080(.a(new_n175), .b(new_n174), .out0(new_n176));
  aoib12aa1n02x5               g081(.a(new_n169), .b(new_n175), .c(new_n174), .out0(new_n177));
  inv040aa1n04x5               g082(.a(new_n169), .o1(new_n178));
  inv000aa1d42x5               g083(.a(new_n171), .o1(new_n179));
  aoai13aa1n02x7               g084(.a(new_n178), .b(new_n179), .c(new_n165), .d(new_n168), .o1(new_n180));
  aoi022aa1n03x5               g085(.a(new_n180), .b(new_n176), .c(new_n173), .d(new_n177), .o1(\s[16] ));
  nano23aa1n02x4               g086(.a(new_n140), .b(new_n146), .c(new_n147), .d(new_n141), .out0(new_n182));
  nona23aa1d18x5               g087(.a(new_n175), .b(new_n170), .c(new_n169), .d(new_n174), .out0(new_n183));
  nano32aa1n03x7               g088(.a(new_n183), .b(new_n164), .c(new_n182), .d(new_n137), .out0(new_n184));
  nanp02aa1n02x5               g089(.a(new_n127), .b(new_n184), .o1(new_n185));
  tech160nm_fiaoi012aa1n03p5x5 g090(.a(new_n97), .b(new_n99), .c(new_n100), .o1(new_n186));
  nanb02aa1n06x5               g091(.a(new_n102), .b(new_n103), .out0(new_n187));
  nanp02aa1n02x5               g092(.a(new_n111), .b(new_n106), .o1(new_n188));
  nor003aa1n03x5               g093(.a(new_n186), .b(new_n187), .c(new_n188), .o1(new_n189));
  nona23aa1n09x5               g094(.a(new_n117), .b(new_n115), .c(new_n114), .d(new_n116), .out0(new_n190));
  xnrc02aa1n02x5               g095(.a(\b[5] ), .b(\a[6] ), .out0(new_n191));
  xnrc02aa1n02x5               g096(.a(\b[4] ), .b(\a[5] ), .out0(new_n192));
  nor003aa1n02x5               g097(.a(new_n190), .b(new_n191), .c(new_n192), .o1(new_n193));
  oai012aa1n06x5               g098(.a(new_n193), .b(new_n189), .c(new_n112), .o1(new_n194));
  nona32aa1n03x5               g099(.a(new_n153), .b(new_n183), .c(new_n162), .d(new_n156), .out0(new_n195));
  norp03aa1n06x5               g100(.a(new_n183), .b(new_n162), .c(new_n156), .o1(new_n196));
  oaoi03aa1n02x5               g101(.a(\a[16] ), .b(\b[15] ), .c(new_n178), .o1(new_n197));
  oabi12aa1n03x5               g102(.a(new_n197), .b(new_n183), .c(new_n168), .out0(new_n198));
  aoi012aa1n12x5               g103(.a(new_n198), .b(new_n155), .c(new_n196), .o1(new_n199));
  aoai13aa1n12x5               g104(.a(new_n199), .b(new_n195), .c(new_n194), .d(new_n126), .o1(new_n200));
  xorc02aa1n12x5               g105(.a(\a[17] ), .b(\b[16] ), .out0(new_n201));
  aoi112aa1n02x5               g106(.a(new_n201), .b(new_n198), .c(new_n155), .d(new_n196), .o1(new_n202));
  aoi022aa1n02x5               g107(.a(new_n200), .b(new_n201), .c(new_n185), .d(new_n202), .o1(\s[17] ));
  inv000aa1d42x5               g108(.a(\a[17] ), .o1(new_n204));
  inv000aa1d42x5               g109(.a(\b[16] ), .o1(new_n205));
  nanp02aa1n02x5               g110(.a(new_n205), .b(new_n204), .o1(new_n206));
  nanp02aa1n02x5               g111(.a(new_n155), .b(new_n196), .o1(new_n207));
  oab012aa1n02x4               g112(.a(new_n197), .b(new_n183), .c(new_n168), .out0(new_n208));
  nanp02aa1n02x5               g113(.a(new_n207), .b(new_n208), .o1(new_n209));
  aoai13aa1n02x5               g114(.a(new_n201), .b(new_n209), .c(new_n127), .d(new_n184), .o1(new_n210));
  nor002aa1n02x5               g115(.a(\b[17] ), .b(\a[18] ), .o1(new_n211));
  nand42aa1d28x5               g116(.a(\b[17] ), .b(\a[18] ), .o1(new_n212));
  norb02aa1n06x4               g117(.a(new_n212), .b(new_n211), .out0(new_n213));
  xnbna2aa1n03x5               g118(.a(new_n213), .b(new_n210), .c(new_n206), .out0(\s[18] ));
  and002aa1n02x5               g119(.a(new_n201), .b(new_n213), .o(new_n215));
  oaoi03aa1n02x5               g120(.a(\a[18] ), .b(\b[17] ), .c(new_n206), .o1(new_n216));
  xorc02aa1n02x5               g121(.a(\a[19] ), .b(\b[18] ), .out0(new_n217));
  aoai13aa1n06x5               g122(.a(new_n217), .b(new_n216), .c(new_n200), .d(new_n215), .o1(new_n218));
  aoi112aa1n02x5               g123(.a(new_n217), .b(new_n216), .c(new_n200), .d(new_n215), .o1(new_n219));
  norb02aa1n03x4               g124(.a(new_n218), .b(new_n219), .out0(\s[19] ));
  xnrc02aa1n02x5               g125(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  xorc02aa1n02x5               g126(.a(\a[20] ), .b(\b[19] ), .out0(new_n222));
  nor042aa1n03x5               g127(.a(\b[18] ), .b(\a[19] ), .o1(new_n223));
  norp02aa1n02x5               g128(.a(new_n222), .b(new_n223), .o1(new_n224));
  tech160nm_fioai012aa1n03p5x5 g129(.a(new_n218), .b(\b[18] ), .c(\a[19] ), .o1(new_n225));
  aoi022aa1n02x7               g130(.a(new_n225), .b(new_n222), .c(new_n218), .d(new_n224), .o1(\s[20] ));
  xnrc02aa1n02x5               g131(.a(\b[19] ), .b(\a[20] ), .out0(new_n227));
  nano32aa1n02x4               g132(.a(new_n227), .b(new_n217), .c(new_n201), .d(new_n213), .out0(new_n228));
  aoi013aa1n02x4               g133(.a(new_n211), .b(new_n212), .c(new_n204), .d(new_n205), .o1(new_n229));
  xnrc02aa1n02x5               g134(.a(\b[18] ), .b(\a[19] ), .out0(new_n230));
  inv000aa1d42x5               g135(.a(\a[20] ), .o1(new_n231));
  inv000aa1d42x5               g136(.a(\b[19] ), .o1(new_n232));
  oaoi03aa1n12x5               g137(.a(new_n231), .b(new_n232), .c(new_n223), .o1(new_n233));
  oai013aa1n02x4               g138(.a(new_n233), .b(new_n229), .c(new_n230), .d(new_n227), .o1(new_n234));
  nor042aa1n04x5               g139(.a(\b[20] ), .b(\a[21] ), .o1(new_n235));
  nanp02aa1n02x5               g140(.a(\b[20] ), .b(\a[21] ), .o1(new_n236));
  nanb02aa1n02x5               g141(.a(new_n235), .b(new_n236), .out0(new_n237));
  inv000aa1d42x5               g142(.a(new_n237), .o1(new_n238));
  aoai13aa1n06x5               g143(.a(new_n238), .b(new_n234), .c(new_n200), .d(new_n228), .o1(new_n239));
  aoi112aa1n02x5               g144(.a(new_n238), .b(new_n234), .c(new_n200), .d(new_n228), .o1(new_n240));
  norb02aa1n03x4               g145(.a(new_n239), .b(new_n240), .out0(\s[21] ));
  nor042aa1n04x5               g146(.a(\b[21] ), .b(\a[22] ), .o1(new_n242));
  nand22aa1n04x5               g147(.a(\b[21] ), .b(\a[22] ), .o1(new_n243));
  nanb02aa1n02x5               g148(.a(new_n242), .b(new_n243), .out0(new_n244));
  inv000aa1d42x5               g149(.a(new_n244), .o1(new_n245));
  aoib12aa1n02x5               g150(.a(new_n235), .b(new_n243), .c(new_n242), .out0(new_n246));
  tech160nm_fioai012aa1n03p5x5 g151(.a(new_n239), .b(\b[20] ), .c(\a[21] ), .o1(new_n247));
  aoi022aa1n02x7               g152(.a(new_n247), .b(new_n245), .c(new_n239), .d(new_n246), .o1(\s[22] ));
  norp02aa1n02x5               g153(.a(new_n227), .b(new_n230), .o1(new_n249));
  nona23aa1n09x5               g154(.a(new_n243), .b(new_n236), .c(new_n235), .d(new_n242), .out0(new_n250));
  nano32aa1n02x4               g155(.a(new_n250), .b(new_n249), .c(new_n213), .d(new_n201), .out0(new_n251));
  nanp03aa1n02x5               g156(.a(new_n216), .b(new_n217), .c(new_n222), .o1(new_n252));
  ao0012aa1n12x5               g157(.a(new_n242), .b(new_n235), .c(new_n243), .o(new_n253));
  inv000aa1d42x5               g158(.a(new_n253), .o1(new_n254));
  aoai13aa1n02x5               g159(.a(new_n254), .b(new_n250), .c(new_n252), .d(new_n233), .o1(new_n255));
  tech160nm_fixorc02aa1n04x5   g160(.a(\a[23] ), .b(\b[22] ), .out0(new_n256));
  aoai13aa1n06x5               g161(.a(new_n256), .b(new_n255), .c(new_n200), .d(new_n251), .o1(new_n257));
  nano23aa1n06x5               g162(.a(new_n235), .b(new_n242), .c(new_n243), .d(new_n236), .out0(new_n258));
  aoi112aa1n02x5               g163(.a(new_n256), .b(new_n253), .c(new_n234), .d(new_n258), .o1(new_n259));
  aobi12aa1n02x5               g164(.a(new_n259), .b(new_n200), .c(new_n251), .out0(new_n260));
  norb02aa1n03x4               g165(.a(new_n257), .b(new_n260), .out0(\s[23] ));
  xorc02aa1n06x5               g166(.a(\a[24] ), .b(\b[23] ), .out0(new_n262));
  norp02aa1n02x5               g167(.a(\b[22] ), .b(\a[23] ), .o1(new_n263));
  norp02aa1n02x5               g168(.a(new_n262), .b(new_n263), .o1(new_n264));
  tech160nm_fioai012aa1n03p5x5 g169(.a(new_n257), .b(\b[22] ), .c(\a[23] ), .o1(new_n265));
  aoi022aa1n02x7               g170(.a(new_n265), .b(new_n262), .c(new_n257), .d(new_n264), .o1(\s[24] ));
  nanp03aa1n02x5               g171(.a(new_n258), .b(new_n256), .c(new_n262), .o1(new_n267));
  nano32aa1n02x5               g172(.a(new_n267), .b(new_n249), .c(new_n213), .d(new_n201), .out0(new_n268));
  aob012aa1n02x5               g173(.a(new_n263), .b(\b[23] ), .c(\a[24] ), .out0(new_n269));
  oai012aa1n02x5               g174(.a(new_n269), .b(\b[23] ), .c(\a[24] ), .o1(new_n270));
  aoi013aa1n06x4               g175(.a(new_n270), .b(new_n253), .c(new_n256), .d(new_n262), .o1(new_n271));
  aoai13aa1n06x5               g176(.a(new_n271), .b(new_n267), .c(new_n252), .d(new_n233), .o1(new_n272));
  xorc02aa1n02x5               g177(.a(\a[25] ), .b(\b[24] ), .out0(new_n273));
  aoai13aa1n06x5               g178(.a(new_n273), .b(new_n272), .c(new_n200), .d(new_n268), .o1(new_n274));
  norp03aa1n02x5               g179(.a(new_n229), .b(new_n230), .c(new_n227), .o1(new_n275));
  inv000aa1n02x5               g180(.a(new_n233), .o1(new_n276));
  nano22aa1n03x7               g181(.a(new_n250), .b(new_n256), .c(new_n262), .out0(new_n277));
  oai012aa1n02x7               g182(.a(new_n277), .b(new_n275), .c(new_n276), .o1(new_n278));
  nano22aa1n02x4               g183(.a(new_n273), .b(new_n278), .c(new_n271), .out0(new_n279));
  aobi12aa1n02x5               g184(.a(new_n279), .b(new_n268), .c(new_n200), .out0(new_n280));
  norb02aa1n03x4               g185(.a(new_n274), .b(new_n280), .out0(\s[25] ));
  xorc02aa1n02x5               g186(.a(\a[26] ), .b(\b[25] ), .out0(new_n282));
  inv000aa1d42x5               g187(.a(\a[25] ), .o1(new_n283));
  aoib12aa1n02x5               g188(.a(new_n282), .b(new_n283), .c(\b[24] ), .out0(new_n284));
  oaib12aa1n06x5               g189(.a(new_n274), .b(\b[24] ), .c(new_n283), .out0(new_n285));
  aoi022aa1n02x7               g190(.a(new_n285), .b(new_n282), .c(new_n274), .d(new_n284), .o1(\s[26] ));
  inv040aa1d32x5               g191(.a(\a[26] ), .o1(new_n287));
  xroi22aa1d06x4               g192(.a(new_n283), .b(\b[24] ), .c(new_n287), .d(\b[25] ), .out0(new_n288));
  nano32aa1n03x7               g193(.a(new_n267), .b(new_n215), .c(new_n288), .d(new_n249), .out0(new_n289));
  aoai13aa1n06x5               g194(.a(new_n289), .b(new_n209), .c(new_n127), .d(new_n184), .o1(new_n290));
  inv000aa1d42x5               g195(.a(new_n288), .o1(new_n291));
  aoi112aa1n02x5               g196(.a(\b[24] ), .b(\a[25] ), .c(\a[26] ), .d(\b[25] ), .o1(new_n292));
  aoib12aa1n02x5               g197(.a(new_n292), .b(new_n287), .c(\b[25] ), .out0(new_n293));
  aoai13aa1n06x5               g198(.a(new_n293), .b(new_n291), .c(new_n278), .d(new_n271), .o1(new_n294));
  xorc02aa1n02x5               g199(.a(\a[27] ), .b(\b[26] ), .out0(new_n295));
  aoai13aa1n06x5               g200(.a(new_n295), .b(new_n294), .c(new_n200), .d(new_n289), .o1(new_n296));
  nanb02aa1n02x5               g201(.a(new_n295), .b(new_n293), .out0(new_n297));
  aoi012aa1n02x5               g202(.a(new_n297), .b(new_n272), .c(new_n288), .o1(new_n298));
  aobi12aa1n02x7               g203(.a(new_n296), .b(new_n298), .c(new_n290), .out0(\s[27] ));
  xorc02aa1n02x5               g204(.a(\a[28] ), .b(\b[27] ), .out0(new_n300));
  norp02aa1n02x5               g205(.a(\b[26] ), .b(\a[27] ), .o1(new_n301));
  norp02aa1n02x5               g206(.a(new_n300), .b(new_n301), .o1(new_n302));
  inv000aa1d42x5               g207(.a(\a[27] ), .o1(new_n303));
  oaib12aa1n06x5               g208(.a(new_n296), .b(\b[26] ), .c(new_n303), .out0(new_n304));
  aoi022aa1n02x7               g209(.a(new_n304), .b(new_n300), .c(new_n296), .d(new_n302), .o1(\s[28] ));
  inv000aa1d42x5               g210(.a(\a[28] ), .o1(new_n306));
  xroi22aa1d04x5               g211(.a(new_n303), .b(\b[26] ), .c(new_n306), .d(\b[27] ), .out0(new_n307));
  aoai13aa1n06x5               g212(.a(new_n307), .b(new_n294), .c(new_n200), .d(new_n289), .o1(new_n308));
  inv000aa1d42x5               g213(.a(\b[27] ), .o1(new_n309));
  oao003aa1n09x5               g214(.a(new_n306), .b(new_n309), .c(new_n301), .carry(new_n310));
  inv000aa1d42x5               g215(.a(new_n310), .o1(new_n311));
  tech160nm_finand02aa1n05x5   g216(.a(new_n308), .b(new_n311), .o1(new_n312));
  xorc02aa1n02x5               g217(.a(\a[29] ), .b(\b[28] ), .out0(new_n313));
  norp02aa1n02x5               g218(.a(new_n310), .b(new_n313), .o1(new_n314));
  aoi022aa1n02x7               g219(.a(new_n312), .b(new_n313), .c(new_n308), .d(new_n314), .o1(\s[29] ));
  xorb03aa1n02x5               g220(.a(new_n99), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  and003aa1n02x5               g221(.a(new_n295), .b(new_n313), .c(new_n300), .o(new_n317));
  aoai13aa1n06x5               g222(.a(new_n317), .b(new_n294), .c(new_n200), .d(new_n289), .o1(new_n318));
  inv000aa1d42x5               g223(.a(\a[29] ), .o1(new_n319));
  inv000aa1d42x5               g224(.a(\b[28] ), .o1(new_n320));
  oaoi03aa1n02x5               g225(.a(new_n319), .b(new_n320), .c(new_n310), .o1(new_n321));
  nand42aa1n04x5               g226(.a(new_n318), .b(new_n321), .o1(new_n322));
  xorc02aa1n02x5               g227(.a(\a[30] ), .b(\b[29] ), .out0(new_n323));
  oabi12aa1n02x5               g228(.a(new_n323), .b(\a[29] ), .c(\b[28] ), .out0(new_n324));
  oaoi13aa1n02x5               g229(.a(new_n324), .b(new_n310), .c(new_n319), .d(new_n320), .o1(new_n325));
  aoi022aa1n02x7               g230(.a(new_n322), .b(new_n323), .c(new_n318), .d(new_n325), .o1(\s[30] ));
  nanb02aa1n02x5               g231(.a(\b[30] ), .b(\a[31] ), .out0(new_n327));
  nanb02aa1n02x5               g232(.a(\a[31] ), .b(\b[30] ), .out0(new_n328));
  and003aa1n02x5               g233(.a(new_n307), .b(new_n323), .c(new_n313), .o(new_n329));
  aoai13aa1n03x5               g234(.a(new_n329), .b(new_n294), .c(new_n200), .d(new_n289), .o1(new_n330));
  oao003aa1n02x5               g235(.a(\a[30] ), .b(\b[29] ), .c(new_n321), .carry(new_n331));
  aoi022aa1n02x7               g236(.a(new_n330), .b(new_n331), .c(new_n328), .d(new_n327), .o1(new_n332));
  aobi12aa1n02x7               g237(.a(new_n293), .b(new_n272), .c(new_n288), .out0(new_n333));
  aobi12aa1n06x5               g238(.a(new_n329), .b(new_n290), .c(new_n333), .out0(new_n334));
  nano32aa1n03x5               g239(.a(new_n334), .b(new_n331), .c(new_n327), .d(new_n328), .out0(new_n335));
  norp02aa1n03x5               g240(.a(new_n332), .b(new_n335), .o1(\s[31] ));
  aoi112aa1n02x5               g241(.a(new_n107), .b(new_n97), .c(new_n99), .d(new_n100), .o1(new_n337));
  aoi012aa1n02x5               g242(.a(new_n337), .b(new_n101), .c(new_n107), .o1(\s[3] ));
  oai013aa1n02x4               g243(.a(new_n113), .b(new_n186), .c(new_n187), .d(new_n188), .o1(new_n339));
  aoi112aa1n02x5               g244(.a(new_n105), .b(new_n104), .c(new_n101), .d(new_n107), .o1(new_n340));
  aoib12aa1n02x5               g245(.a(new_n340), .b(new_n339), .c(new_n102), .out0(\s[4] ));
  xnbna2aa1n03x5               g246(.a(new_n120), .b(new_n108), .c(new_n113), .out0(\s[5] ));
  oai012aa1n02x5               g247(.a(new_n120), .b(new_n189), .c(new_n112), .o1(new_n343));
  xnbna2aa1n03x5               g248(.a(new_n119), .b(new_n343), .c(new_n123), .out0(\s[6] ));
  norb02aa1n02x5               g249(.a(new_n117), .b(new_n116), .out0(new_n345));
  orn002aa1n02x5               g250(.a(\a[6] ), .b(\b[5] ), .o(new_n346));
  aoai13aa1n02x5               g251(.a(new_n119), .b(new_n122), .c(new_n339), .d(new_n120), .o1(new_n347));
  xnbna2aa1n03x5               g252(.a(new_n345), .b(new_n347), .c(new_n346), .out0(\s[7] ));
  norb02aa1n02x5               g253(.a(new_n115), .b(new_n114), .out0(new_n349));
  aob012aa1n02x5               g254(.a(new_n345), .b(new_n347), .c(new_n346), .out0(new_n350));
  oai012aa1n02x5               g255(.a(new_n350), .b(\b[6] ), .c(\a[7] ), .o1(new_n351));
  aoib12aa1n02x5               g256(.a(new_n116), .b(new_n115), .c(new_n114), .out0(new_n352));
  aoi022aa1n02x5               g257(.a(new_n351), .b(new_n349), .c(new_n350), .d(new_n352), .o1(\s[8] ));
  xnbna2aa1n03x5               g258(.a(new_n130), .b(new_n194), .c(new_n126), .out0(\s[9] ));
endmodule


