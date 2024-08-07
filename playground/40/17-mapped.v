// Benchmark "adder" written by ABC on Thu Jul 18 08:35:28 2024

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
    new_n140, new_n141, new_n142, new_n143, new_n145, new_n146, new_n147,
    new_n148, new_n149, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n157, new_n158, new_n159, new_n160, new_n161, new_n162, new_n163,
    new_n165, new_n166, new_n167, new_n168, new_n169, new_n170, new_n171,
    new_n172, new_n173, new_n175, new_n176, new_n177, new_n178, new_n179,
    new_n180, new_n181, new_n182, new_n184, new_n185, new_n186, new_n187,
    new_n188, new_n189, new_n190, new_n191, new_n192, new_n193, new_n194,
    new_n195, new_n196, new_n197, new_n199, new_n200, new_n201, new_n202,
    new_n203, new_n204, new_n205, new_n207, new_n208, new_n209, new_n210,
    new_n211, new_n212, new_n215, new_n216, new_n217, new_n218, new_n220,
    new_n221, new_n222, new_n223, new_n224, new_n225, new_n226, new_n227,
    new_n228, new_n229, new_n230, new_n231, new_n232, new_n234, new_n235,
    new_n236, new_n237, new_n238, new_n239, new_n241, new_n242, new_n243,
    new_n244, new_n245, new_n246, new_n247, new_n248, new_n249, new_n250,
    new_n252, new_n253, new_n254, new_n255, new_n257, new_n258, new_n259,
    new_n260, new_n261, new_n262, new_n263, new_n264, new_n265, new_n266,
    new_n267, new_n268, new_n269, new_n270, new_n272, new_n273, new_n274,
    new_n275, new_n276, new_n278, new_n279, new_n280, new_n281, new_n282,
    new_n283, new_n284, new_n285, new_n286, new_n287, new_n288, new_n290,
    new_n291, new_n292, new_n293, new_n294, new_n295, new_n296, new_n297,
    new_n299, new_n300, new_n301, new_n302, new_n303, new_n304, new_n305,
    new_n306, new_n307, new_n310, new_n311, new_n312, new_n313, new_n314,
    new_n315, new_n316, new_n317, new_n318, new_n320, new_n321, new_n322,
    new_n323, new_n324, new_n325, new_n326, new_n327, new_n328, new_n329,
    new_n332, new_n333, new_n334, new_n337, new_n338, new_n340, new_n341,
    new_n342, new_n344;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor002aa1d32x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  inv000aa1d42x5               g002(.a(new_n97), .o1(new_n98));
  nor042aa1n03x5               g003(.a(\b[1] ), .b(\a[2] ), .o1(new_n99));
  nanp02aa1n12x5               g004(.a(\b[0] ), .b(\a[1] ), .o1(new_n100));
  nanp02aa1n12x5               g005(.a(\b[1] ), .b(\a[2] ), .o1(new_n101));
  aoi012aa1n12x5               g006(.a(new_n99), .b(new_n100), .c(new_n101), .o1(new_n102));
  nor002aa1n06x5               g007(.a(\b[3] ), .b(\a[4] ), .o1(new_n103));
  nanp02aa1n04x5               g008(.a(\b[3] ), .b(\a[4] ), .o1(new_n104));
  norb02aa1n02x5               g009(.a(new_n104), .b(new_n103), .out0(new_n105));
  xorc02aa1n02x5               g010(.a(\a[3] ), .b(\b[2] ), .out0(new_n106));
  nanb03aa1n02x5               g011(.a(new_n102), .b(new_n106), .c(new_n105), .out0(new_n107));
  inv040aa1d32x5               g012(.a(\a[3] ), .o1(new_n108));
  inv040aa1d28x5               g013(.a(\b[2] ), .o1(new_n109));
  nand42aa1n20x5               g014(.a(new_n109), .b(new_n108), .o1(new_n110));
  oaoi03aa1n09x5               g015(.a(\a[4] ), .b(\b[3] ), .c(new_n110), .o1(new_n111));
  inv000aa1n02x5               g016(.a(new_n111), .o1(new_n112));
  nor042aa1n06x5               g017(.a(\b[7] ), .b(\a[8] ), .o1(new_n113));
  nand02aa1n06x5               g018(.a(\b[7] ), .b(\a[8] ), .o1(new_n114));
  nor002aa1d32x5               g019(.a(\b[6] ), .b(\a[7] ), .o1(new_n115));
  nanp02aa1n04x5               g020(.a(\b[6] ), .b(\a[7] ), .o1(new_n116));
  nano23aa1n09x5               g021(.a(new_n113), .b(new_n115), .c(new_n116), .d(new_n114), .out0(new_n117));
  nor042aa1d18x5               g022(.a(\b[5] ), .b(\a[6] ), .o1(new_n118));
  nand22aa1n03x5               g023(.a(\b[5] ), .b(\a[6] ), .o1(new_n119));
  nanb02aa1n03x5               g024(.a(new_n118), .b(new_n119), .out0(new_n120));
  xnrc02aa1n02x5               g025(.a(\b[4] ), .b(\a[5] ), .out0(new_n121));
  nona22aa1n02x4               g026(.a(new_n117), .b(new_n121), .c(new_n120), .out0(new_n122));
  inv000aa1d42x5               g027(.a(new_n118), .o1(new_n123));
  norp02aa1n02x5               g028(.a(\b[4] ), .b(\a[5] ), .o1(new_n124));
  aob012aa1n02x5               g029(.a(new_n123), .b(new_n124), .c(new_n119), .out0(new_n125));
  inv000aa1d42x5               g030(.a(new_n115), .o1(new_n126));
  oaoi03aa1n02x5               g031(.a(\a[8] ), .b(\b[7] ), .c(new_n126), .o1(new_n127));
  aoi012aa1n12x5               g032(.a(new_n127), .b(new_n117), .c(new_n125), .o1(new_n128));
  aoai13aa1n06x5               g033(.a(new_n128), .b(new_n122), .c(new_n107), .d(new_n112), .o1(new_n129));
  xnrc02aa1n12x5               g034(.a(\b[8] ), .b(\a[9] ), .out0(new_n130));
  nanb02aa1n02x5               g035(.a(new_n130), .b(new_n129), .out0(new_n131));
  nor042aa1n06x5               g036(.a(\b[9] ), .b(\a[10] ), .o1(new_n132));
  nand02aa1d28x5               g037(.a(\b[9] ), .b(\a[10] ), .o1(new_n133));
  norb02aa1n12x5               g038(.a(new_n133), .b(new_n132), .out0(new_n134));
  xnbna2aa1n03x5               g039(.a(new_n134), .b(new_n131), .c(new_n98), .out0(\s[10] ));
  norb02aa1d21x5               g040(.a(new_n134), .b(new_n130), .out0(new_n136));
  aoi012aa1d24x5               g041(.a(new_n132), .b(new_n97), .c(new_n133), .o1(new_n137));
  inv000aa1d42x5               g042(.a(new_n137), .o1(new_n138));
  nor002aa1n12x5               g043(.a(\b[10] ), .b(\a[11] ), .o1(new_n139));
  nanp02aa1n12x5               g044(.a(\b[10] ), .b(\a[11] ), .o1(new_n140));
  norb02aa1n02x5               g045(.a(new_n140), .b(new_n139), .out0(new_n141));
  aoai13aa1n02x5               g046(.a(new_n141), .b(new_n138), .c(new_n129), .d(new_n136), .o1(new_n142));
  aoi112aa1n02x5               g047(.a(new_n141), .b(new_n138), .c(new_n129), .d(new_n136), .o1(new_n143));
  norb02aa1n02x5               g048(.a(new_n142), .b(new_n143), .out0(\s[11] ));
  nor022aa1n16x5               g049(.a(\b[11] ), .b(\a[12] ), .o1(new_n145));
  nand02aa1d06x5               g050(.a(\b[11] ), .b(\a[12] ), .o1(new_n146));
  norb02aa1n02x5               g051(.a(new_n146), .b(new_n145), .out0(new_n147));
  aoib12aa1n02x5               g052(.a(new_n139), .b(new_n146), .c(new_n145), .out0(new_n148));
  oai012aa1n02x5               g053(.a(new_n142), .b(\b[10] ), .c(\a[11] ), .o1(new_n149));
  aoi022aa1n02x5               g054(.a(new_n149), .b(new_n147), .c(new_n142), .d(new_n148), .o1(\s[12] ));
  nano32aa1n02x4               g055(.a(new_n130), .b(new_n147), .c(new_n134), .d(new_n141), .out0(new_n151));
  nano23aa1n02x4               g056(.a(new_n139), .b(new_n145), .c(new_n146), .d(new_n140), .out0(new_n152));
  tech160nm_fiaoi012aa1n03p5x5 g057(.a(new_n145), .b(new_n139), .c(new_n146), .o1(new_n153));
  aobi12aa1n02x5               g058(.a(new_n153), .b(new_n152), .c(new_n138), .out0(new_n154));
  aob012aa1n02x5               g059(.a(new_n154), .b(new_n129), .c(new_n151), .out0(new_n155));
  xorb03aa1n02x5               g060(.a(new_n155), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  nor002aa1d32x5               g061(.a(\b[12] ), .b(\a[13] ), .o1(new_n157));
  nand22aa1n12x5               g062(.a(\b[12] ), .b(\a[13] ), .o1(new_n158));
  nor002aa1d32x5               g063(.a(\b[13] ), .b(\a[14] ), .o1(new_n159));
  nand02aa1d16x5               g064(.a(\b[13] ), .b(\a[14] ), .o1(new_n160));
  norb02aa1n02x5               g065(.a(new_n160), .b(new_n159), .out0(new_n161));
  aoi112aa1n02x5               g066(.a(new_n157), .b(new_n161), .c(new_n155), .d(new_n158), .o1(new_n162));
  aoai13aa1n02x5               g067(.a(new_n161), .b(new_n157), .c(new_n155), .d(new_n158), .o1(new_n163));
  norb02aa1n02x5               g068(.a(new_n163), .b(new_n162), .out0(\s[14] ));
  nona23aa1d18x5               g069(.a(new_n146), .b(new_n140), .c(new_n139), .d(new_n145), .out0(new_n165));
  tech160nm_fioai012aa1n04x5   g070(.a(new_n153), .b(new_n165), .c(new_n137), .o1(new_n166));
  nona23aa1n09x5               g071(.a(new_n160), .b(new_n158), .c(new_n157), .d(new_n159), .out0(new_n167));
  inv040aa1n02x5               g072(.a(new_n167), .o1(new_n168));
  aoai13aa1n03x5               g073(.a(new_n168), .b(new_n166), .c(new_n129), .d(new_n151), .o1(new_n169));
  tech160nm_fiaoi012aa1n03p5x5 g074(.a(new_n159), .b(new_n157), .c(new_n160), .o1(new_n170));
  nor002aa1d32x5               g075(.a(\b[14] ), .b(\a[15] ), .o1(new_n171));
  nanp02aa1n04x5               g076(.a(\b[14] ), .b(\a[15] ), .o1(new_n172));
  norb02aa1n09x5               g077(.a(new_n172), .b(new_n171), .out0(new_n173));
  xnbna2aa1n03x5               g078(.a(new_n173), .b(new_n169), .c(new_n170), .out0(\s[15] ));
  aob012aa1n03x5               g079(.a(new_n173), .b(new_n169), .c(new_n170), .out0(new_n175));
  nor022aa1n16x5               g080(.a(\b[15] ), .b(\a[16] ), .o1(new_n176));
  nand02aa1d06x5               g081(.a(\b[15] ), .b(\a[16] ), .o1(new_n177));
  norb02aa1n02x5               g082(.a(new_n177), .b(new_n176), .out0(new_n178));
  aoib12aa1n02x5               g083(.a(new_n171), .b(new_n177), .c(new_n176), .out0(new_n179));
  inv000aa1d42x5               g084(.a(new_n171), .o1(new_n180));
  inv000aa1d42x5               g085(.a(new_n173), .o1(new_n181));
  aoai13aa1n02x5               g086(.a(new_n180), .b(new_n181), .c(new_n169), .d(new_n170), .o1(new_n182));
  aoi022aa1n03x5               g087(.a(new_n182), .b(new_n178), .c(new_n175), .d(new_n179), .o1(\s[16] ));
  nanb02aa1n03x5               g088(.a(new_n103), .b(new_n104), .out0(new_n184));
  nand42aa1n03x5               g089(.a(\b[2] ), .b(\a[3] ), .o1(new_n185));
  nand42aa1n04x5               g090(.a(new_n110), .b(new_n185), .o1(new_n186));
  nor003aa1n02x5               g091(.a(new_n102), .b(new_n184), .c(new_n186), .o1(new_n187));
  nona23aa1n09x5               g092(.a(new_n116), .b(new_n114), .c(new_n113), .d(new_n115), .out0(new_n188));
  nor003aa1n02x5               g093(.a(new_n188), .b(new_n120), .c(new_n121), .o1(new_n189));
  oai012aa1n04x7               g094(.a(new_n189), .b(new_n187), .c(new_n111), .o1(new_n190));
  nona23aa1d18x5               g095(.a(new_n177), .b(new_n172), .c(new_n171), .d(new_n176), .out0(new_n191));
  nona23aa1d16x5               g096(.a(new_n168), .b(new_n136), .c(new_n191), .d(new_n165), .out0(new_n192));
  nor002aa1n02x5               g097(.a(new_n191), .b(new_n167), .o1(new_n193));
  aoi012aa1n02x7               g098(.a(new_n176), .b(new_n171), .c(new_n177), .o1(new_n194));
  oai012aa1n03x5               g099(.a(new_n194), .b(new_n191), .c(new_n170), .o1(new_n195));
  aoi012aa1n06x5               g100(.a(new_n195), .b(new_n166), .c(new_n193), .o1(new_n196));
  aoai13aa1n12x5               g101(.a(new_n196), .b(new_n192), .c(new_n190), .d(new_n128), .o1(new_n197));
  xorb03aa1n02x5               g102(.a(new_n197), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  nor042aa1n09x5               g103(.a(\b[16] ), .b(\a[17] ), .o1(new_n199));
  tech160nm_fixorc02aa1n02p5x5 g104(.a(\a[17] ), .b(\b[16] ), .out0(new_n200));
  nor042aa1n04x5               g105(.a(\b[17] ), .b(\a[18] ), .o1(new_n201));
  nand02aa1n16x5               g106(.a(\b[17] ), .b(\a[18] ), .o1(new_n202));
  norb02aa1n06x4               g107(.a(new_n202), .b(new_n201), .out0(new_n203));
  aoi112aa1n02x5               g108(.a(new_n199), .b(new_n203), .c(new_n197), .d(new_n200), .o1(new_n204));
  aoai13aa1n02x5               g109(.a(new_n203), .b(new_n199), .c(new_n197), .d(new_n200), .o1(new_n205));
  norb02aa1n02x7               g110(.a(new_n205), .b(new_n204), .out0(\s[18] ));
  and002aa1n02x5               g111(.a(new_n200), .b(new_n203), .o(new_n207));
  aoi012aa1d18x5               g112(.a(new_n201), .b(new_n199), .c(new_n202), .o1(new_n208));
  inv000aa1d42x5               g113(.a(new_n208), .o1(new_n209));
  xorc02aa1n02x5               g114(.a(\a[19] ), .b(\b[18] ), .out0(new_n210));
  aoai13aa1n06x5               g115(.a(new_n210), .b(new_n209), .c(new_n197), .d(new_n207), .o1(new_n211));
  aoi112aa1n02x5               g116(.a(new_n210), .b(new_n209), .c(new_n197), .d(new_n207), .o1(new_n212));
  norb02aa1n02x7               g117(.a(new_n211), .b(new_n212), .out0(\s[19] ));
  xnrc02aa1n02x5               g118(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  xorc02aa1n02x5               g119(.a(\a[20] ), .b(\b[19] ), .out0(new_n215));
  norp02aa1n02x5               g120(.a(\b[18] ), .b(\a[19] ), .o1(new_n216));
  norp02aa1n02x5               g121(.a(new_n215), .b(new_n216), .o1(new_n217));
  tech160nm_fioai012aa1n03p5x5 g122(.a(new_n211), .b(\b[18] ), .c(\a[19] ), .o1(new_n218));
  aoi022aa1n02x7               g123(.a(new_n218), .b(new_n215), .c(new_n211), .d(new_n217), .o1(\s[20] ));
  xnrc02aa1n03x5               g124(.a(\b[19] ), .b(\a[20] ), .out0(new_n220));
  nano32aa1n02x4               g125(.a(new_n220), .b(new_n210), .c(new_n200), .d(new_n203), .out0(new_n221));
  xnrc02aa1n02x5               g126(.a(\b[18] ), .b(\a[19] ), .out0(new_n222));
  norp02aa1n02x5               g127(.a(\b[19] ), .b(\a[20] ), .o1(new_n223));
  nanp02aa1n02x5               g128(.a(\b[19] ), .b(\a[20] ), .o1(new_n224));
  aoi012aa1n02x5               g129(.a(new_n223), .b(new_n216), .c(new_n224), .o1(new_n225));
  oai013aa1n03x5               g130(.a(new_n225), .b(new_n222), .c(new_n220), .d(new_n208), .o1(new_n226));
  nor042aa1n03x5               g131(.a(\b[20] ), .b(\a[21] ), .o1(new_n227));
  nanp02aa1n04x5               g132(.a(\b[20] ), .b(\a[21] ), .o1(new_n228));
  nanb02aa1n18x5               g133(.a(new_n227), .b(new_n228), .out0(new_n229));
  inv000aa1d42x5               g134(.a(new_n229), .o1(new_n230));
  aoai13aa1n06x5               g135(.a(new_n230), .b(new_n226), .c(new_n197), .d(new_n221), .o1(new_n231));
  aoi112aa1n02x5               g136(.a(new_n230), .b(new_n226), .c(new_n197), .d(new_n221), .o1(new_n232));
  norb02aa1n03x4               g137(.a(new_n231), .b(new_n232), .out0(\s[21] ));
  nor002aa1n06x5               g138(.a(\b[21] ), .b(\a[22] ), .o1(new_n234));
  nand22aa1n09x5               g139(.a(\b[21] ), .b(\a[22] ), .o1(new_n235));
  nanb02aa1d24x5               g140(.a(new_n234), .b(new_n235), .out0(new_n236));
  inv000aa1d42x5               g141(.a(new_n236), .o1(new_n237));
  aoib12aa1n02x5               g142(.a(new_n227), .b(new_n235), .c(new_n234), .out0(new_n238));
  tech160nm_fioai012aa1n03p5x5 g143(.a(new_n231), .b(\b[20] ), .c(\a[21] ), .o1(new_n239));
  aoi022aa1n02x7               g144(.a(new_n239), .b(new_n237), .c(new_n231), .d(new_n238), .o1(\s[22] ));
  norp02aa1n02x5               g145(.a(new_n220), .b(new_n222), .o1(new_n241));
  nona23aa1d18x5               g146(.a(new_n235), .b(new_n228), .c(new_n227), .d(new_n234), .out0(new_n242));
  nano32aa1n02x4               g147(.a(new_n242), .b(new_n241), .c(new_n203), .d(new_n200), .out0(new_n243));
  inv000aa1d42x5               g148(.a(new_n242), .o1(new_n244));
  ao0012aa1n03x7               g149(.a(new_n234), .b(new_n227), .c(new_n235), .o(new_n245));
  tech160nm_fiao0012aa1n02p5x5 g150(.a(new_n245), .b(new_n226), .c(new_n244), .o(new_n246));
  xnrc02aa1n12x5               g151(.a(\b[22] ), .b(\a[23] ), .out0(new_n247));
  inv000aa1d42x5               g152(.a(new_n247), .o1(new_n248));
  aoai13aa1n06x5               g153(.a(new_n248), .b(new_n246), .c(new_n197), .d(new_n243), .o1(new_n249));
  aoi112aa1n02x5               g154(.a(new_n248), .b(new_n246), .c(new_n197), .d(new_n243), .o1(new_n250));
  norb02aa1n03x4               g155(.a(new_n249), .b(new_n250), .out0(\s[23] ));
  xorc02aa1n12x5               g156(.a(\a[24] ), .b(\b[23] ), .out0(new_n252));
  norp02aa1n02x5               g157(.a(\b[22] ), .b(\a[23] ), .o1(new_n253));
  norp02aa1n02x5               g158(.a(new_n252), .b(new_n253), .o1(new_n254));
  tech160nm_fioai012aa1n03p5x5 g159(.a(new_n249), .b(\b[22] ), .c(\a[23] ), .o1(new_n255));
  aoi022aa1n02x7               g160(.a(new_n255), .b(new_n252), .c(new_n249), .d(new_n254), .o1(\s[24] ));
  nona23aa1n09x5               g161(.a(new_n248), .b(new_n252), .c(new_n236), .d(new_n229), .out0(new_n257));
  nano32aa1n02x5               g162(.a(new_n257), .b(new_n241), .c(new_n203), .d(new_n200), .out0(new_n258));
  and002aa1n03x5               g163(.a(new_n197), .b(new_n258), .o(new_n259));
  nanb03aa1n02x5               g164(.a(new_n208), .b(new_n215), .c(new_n210), .out0(new_n260));
  aob012aa1n02x5               g165(.a(new_n253), .b(\b[23] ), .c(\a[24] ), .out0(new_n261));
  oai012aa1n02x5               g166(.a(new_n261), .b(\b[23] ), .c(\a[24] ), .o1(new_n262));
  aoi013aa1n03x5               g167(.a(new_n262), .b(new_n248), .c(new_n245), .d(new_n252), .o1(new_n263));
  aoai13aa1n06x5               g168(.a(new_n263), .b(new_n257), .c(new_n260), .d(new_n225), .o1(new_n264));
  xorc02aa1n02x5               g169(.a(\a[25] ), .b(\b[24] ), .out0(new_n265));
  aoai13aa1n06x5               g170(.a(new_n265), .b(new_n264), .c(new_n197), .d(new_n258), .o1(new_n266));
  xnrc02aa1n02x5               g171(.a(\b[23] ), .b(\a[24] ), .out0(new_n267));
  nor043aa1n04x5               g172(.a(new_n242), .b(new_n247), .c(new_n267), .o1(new_n268));
  nand42aa1n04x5               g173(.a(new_n226), .b(new_n268), .o1(new_n269));
  nanb03aa1n02x5               g174(.a(new_n265), .b(new_n269), .c(new_n263), .out0(new_n270));
  oa0012aa1n03x5               g175(.a(new_n266), .b(new_n259), .c(new_n270), .o(\s[25] ));
  xorc02aa1n02x5               g176(.a(\a[26] ), .b(\b[25] ), .out0(new_n272));
  norp02aa1n02x5               g177(.a(\b[24] ), .b(\a[25] ), .o1(new_n273));
  norp02aa1n02x5               g178(.a(new_n272), .b(new_n273), .o1(new_n274));
  inv000aa1d42x5               g179(.a(\a[25] ), .o1(new_n275));
  oaib12aa1n02x7               g180(.a(new_n266), .b(\b[24] ), .c(new_n275), .out0(new_n276));
  aoi022aa1n02x7               g181(.a(new_n276), .b(new_n272), .c(new_n266), .d(new_n274), .o1(\s[26] ));
  inv000aa1d42x5               g182(.a(new_n192), .o1(new_n278));
  oa0012aa1n02x5               g183(.a(new_n194), .b(new_n191), .c(new_n170), .o(new_n279));
  oaib12aa1n02x5               g184(.a(new_n279), .b(new_n154), .c(new_n193), .out0(new_n280));
  inv040aa1d32x5               g185(.a(\a[26] ), .o1(new_n281));
  xroi22aa1d06x4               g186(.a(new_n275), .b(\b[24] ), .c(new_n281), .d(\b[25] ), .out0(new_n282));
  nano32aa1n03x7               g187(.a(new_n257), .b(new_n207), .c(new_n282), .d(new_n241), .out0(new_n283));
  aoai13aa1n06x5               g188(.a(new_n283), .b(new_n280), .c(new_n129), .d(new_n278), .o1(new_n284));
  inv000aa1d42x5               g189(.a(\b[25] ), .o1(new_n285));
  oaoi03aa1n02x5               g190(.a(new_n281), .b(new_n285), .c(new_n273), .o1(new_n286));
  aobi12aa1n06x5               g191(.a(new_n286), .b(new_n264), .c(new_n282), .out0(new_n287));
  xorc02aa1n02x5               g192(.a(\a[27] ), .b(\b[26] ), .out0(new_n288));
  xnbna2aa1n03x5               g193(.a(new_n288), .b(new_n284), .c(new_n287), .out0(\s[27] ));
  inv000aa1d42x5               g194(.a(new_n282), .o1(new_n290));
  aoai13aa1n04x5               g195(.a(new_n286), .b(new_n290), .c(new_n269), .d(new_n263), .o1(new_n291));
  aoai13aa1n03x5               g196(.a(new_n288), .b(new_n291), .c(new_n197), .d(new_n283), .o1(new_n292));
  xorc02aa1n02x5               g197(.a(\a[28] ), .b(\b[27] ), .out0(new_n293));
  norp02aa1n02x5               g198(.a(\b[26] ), .b(\a[27] ), .o1(new_n294));
  norp02aa1n02x5               g199(.a(new_n293), .b(new_n294), .o1(new_n295));
  inv000aa1d42x5               g200(.a(\a[27] ), .o1(new_n296));
  oaib12aa1n06x5               g201(.a(new_n292), .b(\b[26] ), .c(new_n296), .out0(new_n297));
  aoi022aa1n02x7               g202(.a(new_n297), .b(new_n293), .c(new_n292), .d(new_n295), .o1(\s[28] ));
  inv000aa1d42x5               g203(.a(\a[28] ), .o1(new_n299));
  xroi22aa1d04x5               g204(.a(new_n296), .b(\b[26] ), .c(new_n299), .d(\b[27] ), .out0(new_n300));
  aoai13aa1n03x5               g205(.a(new_n300), .b(new_n291), .c(new_n197), .d(new_n283), .o1(new_n301));
  inv000aa1d42x5               g206(.a(\b[27] ), .o1(new_n302));
  oao003aa1n09x5               g207(.a(new_n299), .b(new_n302), .c(new_n294), .carry(new_n303));
  inv000aa1d42x5               g208(.a(new_n303), .o1(new_n304));
  nand42aa1n02x5               g209(.a(new_n301), .b(new_n304), .o1(new_n305));
  xorc02aa1n02x5               g210(.a(\a[29] ), .b(\b[28] ), .out0(new_n306));
  norp02aa1n02x5               g211(.a(new_n303), .b(new_n306), .o1(new_n307));
  aoi022aa1n02x7               g212(.a(new_n305), .b(new_n306), .c(new_n301), .d(new_n307), .o1(\s[29] ));
  xorb03aa1n02x5               g213(.a(new_n100), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  and003aa1n02x5               g214(.a(new_n288), .b(new_n306), .c(new_n293), .o(new_n310));
  aoai13aa1n03x5               g215(.a(new_n310), .b(new_n291), .c(new_n197), .d(new_n283), .o1(new_n311));
  inv000aa1d42x5               g216(.a(\a[29] ), .o1(new_n312));
  inv000aa1d42x5               g217(.a(\b[28] ), .o1(new_n313));
  oaoi03aa1n02x5               g218(.a(new_n312), .b(new_n313), .c(new_n303), .o1(new_n314));
  nand42aa1n02x5               g219(.a(new_n311), .b(new_n314), .o1(new_n315));
  xorc02aa1n02x5               g220(.a(\a[30] ), .b(\b[29] ), .out0(new_n316));
  oabi12aa1n02x5               g221(.a(new_n316), .b(\a[29] ), .c(\b[28] ), .out0(new_n317));
  oaoi13aa1n02x5               g222(.a(new_n317), .b(new_n303), .c(new_n312), .d(new_n313), .o1(new_n318));
  aoi022aa1n02x7               g223(.a(new_n315), .b(new_n316), .c(new_n311), .d(new_n318), .o1(\s[30] ));
  nanb02aa1n02x5               g224(.a(\b[30] ), .b(\a[31] ), .out0(new_n320));
  nanb02aa1n02x5               g225(.a(\a[31] ), .b(\b[30] ), .out0(new_n321));
  and003aa1n02x5               g226(.a(new_n300), .b(new_n316), .c(new_n306), .o(new_n322));
  aoai13aa1n03x5               g227(.a(new_n322), .b(new_n291), .c(new_n197), .d(new_n283), .o1(new_n323));
  oao003aa1n02x5               g228(.a(\a[30] ), .b(\b[29] ), .c(new_n314), .carry(new_n324));
  aoi022aa1n02x7               g229(.a(new_n323), .b(new_n324), .c(new_n321), .d(new_n320), .o1(new_n325));
  xnrc02aa1n02x5               g230(.a(\b[30] ), .b(\a[31] ), .out0(new_n326));
  inv000aa1d42x5               g231(.a(new_n322), .o1(new_n327));
  tech160nm_fiaoi012aa1n02p5x5 g232(.a(new_n327), .b(new_n284), .c(new_n287), .o1(new_n328));
  nano22aa1n03x5               g233(.a(new_n328), .b(new_n326), .c(new_n324), .out0(new_n329));
  norp02aa1n03x5               g234(.a(new_n325), .b(new_n329), .o1(\s[31] ));
  xnbna2aa1n03x5               g235(.a(new_n102), .b(new_n185), .c(new_n110), .out0(\s[3] ));
  oai013aa1n02x4               g236(.a(new_n112), .b(new_n102), .c(new_n184), .d(new_n186), .o1(new_n332));
  oaib12aa1n02x5               g237(.a(new_n110), .b(new_n103), .c(new_n104), .out0(new_n333));
  oab012aa1n02x4               g238(.a(new_n333), .b(new_n102), .c(new_n186), .out0(new_n334));
  oaoi13aa1n02x5               g239(.a(new_n334), .b(new_n332), .c(\a[4] ), .d(\b[3] ), .o1(\s[4] ));
  xorb03aa1n02x5               g240(.a(new_n332), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  orn002aa1n02x5               g241(.a(\a[5] ), .b(\b[4] ), .o(new_n337));
  aoai13aa1n02x5               g242(.a(new_n337), .b(new_n121), .c(new_n107), .d(new_n112), .o1(new_n338));
  xorb03aa1n02x5               g243(.a(new_n338), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  norb02aa1n02x5               g244(.a(new_n116), .b(new_n115), .out0(new_n340));
  aoai13aa1n02x5               g245(.a(new_n340), .b(new_n118), .c(new_n338), .d(new_n119), .o1(new_n341));
  aoi112aa1n02x5               g246(.a(new_n340), .b(new_n118), .c(new_n338), .d(new_n119), .o1(new_n342));
  norb02aa1n02x5               g247(.a(new_n341), .b(new_n342), .out0(\s[7] ));
  norb02aa1n02x5               g248(.a(new_n114), .b(new_n113), .out0(new_n344));
  xnbna2aa1n03x5               g249(.a(new_n344), .b(new_n341), .c(new_n126), .out0(\s[8] ));
  xobna2aa1n03x5               g250(.a(new_n130), .b(new_n190), .c(new_n128), .out0(\s[9] ));
endmodule


