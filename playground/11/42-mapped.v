// Benchmark "adder" written by ABC on Wed Jul 17 17:58:27 2024

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
    new_n141, new_n142, new_n143, new_n144, new_n145, new_n146, new_n147,
    new_n148, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n159, new_n161, new_n162, new_n163,
    new_n164, new_n165, new_n167, new_n168, new_n169, new_n170, new_n171,
    new_n172, new_n173, new_n174, new_n175, new_n177, new_n178, new_n179,
    new_n180, new_n181, new_n182, new_n184, new_n185, new_n186, new_n187,
    new_n188, new_n189, new_n190, new_n191, new_n192, new_n193, new_n194,
    new_n196, new_n197, new_n198, new_n199, new_n200, new_n201, new_n202,
    new_n203, new_n204, new_n205, new_n206, new_n207, new_n208, new_n210,
    new_n211, new_n212, new_n213, new_n214, new_n215, new_n216, new_n217,
    new_n220, new_n221, new_n222, new_n223, new_n224, new_n225, new_n227,
    new_n228, new_n229, new_n230, new_n231, new_n232, new_n233, new_n234,
    new_n235, new_n236, new_n238, new_n239, new_n240, new_n241, new_n242,
    new_n243, new_n244, new_n246, new_n247, new_n248, new_n249, new_n250,
    new_n251, new_n252, new_n253, new_n254, new_n256, new_n257, new_n258,
    new_n259, new_n260, new_n261, new_n262, new_n263, new_n264, new_n266,
    new_n267, new_n268, new_n269, new_n270, new_n271, new_n272, new_n273,
    new_n274, new_n275, new_n277, new_n278, new_n279, new_n280, new_n281,
    new_n282, new_n283, new_n284, new_n286, new_n287, new_n288, new_n289,
    new_n290, new_n291, new_n292, new_n294, new_n295, new_n296, new_n297,
    new_n298, new_n299, new_n300, new_n301, new_n302, new_n303, new_n305,
    new_n306, new_n307, new_n308, new_n309, new_n310, new_n311, new_n313,
    new_n315, new_n316, new_n317, new_n318, new_n319, new_n320, new_n321,
    new_n322, new_n323, new_n324, new_n326, new_n327, new_n328, new_n329,
    new_n330, new_n331, new_n332, new_n335, new_n336, new_n338, new_n340,
    new_n342, new_n344;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  norp02aa1n02x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  norp02aa1n04x5               g002(.a(\b[2] ), .b(\a[3] ), .o1(new_n98));
  norp02aa1n02x5               g003(.a(\b[3] ), .b(\a[4] ), .o1(new_n99));
  norp02aa1n02x5               g004(.a(new_n99), .b(new_n98), .o1(new_n100));
  nanp02aa1n02x5               g005(.a(\b[2] ), .b(\a[3] ), .o1(new_n101));
  nanb02aa1n02x5               g006(.a(new_n98), .b(new_n101), .out0(new_n102));
  nanp02aa1n02x5               g007(.a(\b[1] ), .b(\a[2] ), .o1(new_n103));
  oai112aa1n02x7               g008(.a(\a[1] ), .b(\b[0] ), .c(\b[1] ), .d(\a[2] ), .o1(new_n104));
  nanp02aa1n02x5               g009(.a(new_n104), .b(new_n103), .o1(new_n105));
  oai012aa1n04x7               g010(.a(new_n100), .b(new_n105), .c(new_n102), .o1(new_n106));
  nanp02aa1n02x5               g011(.a(\b[3] ), .b(\a[4] ), .o1(new_n107));
  nanp02aa1n02x5               g012(.a(\b[4] ), .b(\a[5] ), .o1(new_n108));
  nor022aa1n06x5               g013(.a(\b[4] ), .b(\a[5] ), .o1(new_n109));
  nanp02aa1n02x5               g014(.a(\b[7] ), .b(\a[8] ), .o1(new_n110));
  nanb03aa1n02x5               g015(.a(new_n109), .b(new_n110), .c(new_n108), .out0(new_n111));
  inv000aa1d42x5               g016(.a(\a[8] ), .o1(new_n112));
  inv000aa1d42x5               g017(.a(\b[7] ), .o1(new_n113));
  norp02aa1n04x5               g018(.a(\b[5] ), .b(\a[6] ), .o1(new_n114));
  aoi012aa1n02x5               g019(.a(new_n114), .b(new_n112), .c(new_n113), .o1(new_n115));
  nanp02aa1n02x5               g020(.a(\b[5] ), .b(\a[6] ), .o1(new_n116));
  nor022aa1n08x5               g021(.a(\b[6] ), .b(\a[7] ), .o1(new_n117));
  nand42aa1n02x5               g022(.a(\b[6] ), .b(\a[7] ), .o1(new_n118));
  nanb03aa1n02x5               g023(.a(new_n117), .b(new_n118), .c(new_n116), .out0(new_n119));
  nano23aa1n03x7               g024(.a(new_n111), .b(new_n119), .c(new_n115), .d(new_n107), .out0(new_n120));
  nanp02aa1n02x5               g025(.a(new_n120), .b(new_n106), .o1(new_n121));
  nanp02aa1n02x5               g026(.a(new_n113), .b(new_n112), .o1(new_n122));
  inv000aa1n02x5               g027(.a(new_n117), .o1(new_n123));
  tech160nm_fioai012aa1n05x5   g028(.a(new_n116), .b(new_n114), .c(new_n109), .o1(new_n124));
  nanp02aa1n02x5               g029(.a(new_n118), .b(new_n110), .o1(new_n125));
  aoai13aa1n12x5               g030(.a(new_n122), .b(new_n125), .c(new_n124), .d(new_n123), .o1(new_n126));
  inv000aa1d42x5               g031(.a(new_n126), .o1(new_n127));
  nanp02aa1n02x5               g032(.a(new_n121), .b(new_n127), .o1(new_n128));
  xorc02aa1n02x5               g033(.a(\a[9] ), .b(\b[8] ), .out0(new_n129));
  tech160nm_fixnrc02aa1n05x5   g034(.a(\b[9] ), .b(\a[10] ), .out0(new_n130));
  aoai13aa1n02x5               g035(.a(new_n130), .b(new_n97), .c(new_n128), .d(new_n129), .o1(new_n131));
  aoi012aa1n06x5               g036(.a(new_n126), .b(new_n120), .c(new_n106), .o1(new_n132));
  nor042aa1n02x5               g037(.a(new_n130), .b(new_n97), .o1(new_n133));
  oaib12aa1n02x5               g038(.a(new_n133), .b(new_n132), .c(new_n129), .out0(new_n134));
  nanp02aa1n02x5               g039(.a(new_n131), .b(new_n134), .o1(\s[10] ));
  nanp02aa1n02x5               g040(.a(\b[9] ), .b(\a[10] ), .o1(new_n136));
  xorc02aa1n02x5               g041(.a(\a[10] ), .b(\b[9] ), .out0(new_n137));
  nanp02aa1n02x5               g042(.a(new_n137), .b(new_n129), .o1(new_n138));
  obai22aa1n02x7               g043(.a(new_n136), .b(new_n133), .c(new_n132), .d(new_n138), .out0(new_n139));
  xorb03aa1n02x5               g044(.a(new_n139), .b(\b[10] ), .c(\a[11] ), .out0(\s[11] ));
  xorc02aa1n02x5               g045(.a(\a[12] ), .b(\b[11] ), .out0(new_n141));
  inv000aa1d42x5               g046(.a(\a[11] ), .o1(new_n142));
  inv000aa1d42x5               g047(.a(\b[10] ), .o1(new_n143));
  oaoi03aa1n02x5               g048(.a(new_n142), .b(new_n143), .c(new_n139), .o1(new_n144));
  oai022aa1n02x5               g049(.a(\a[11] ), .b(\b[10] ), .c(\b[11] ), .d(\a[12] ), .o1(new_n145));
  xorc02aa1n02x5               g050(.a(\a[11] ), .b(\b[10] ), .out0(new_n146));
  nanp02aa1n02x5               g051(.a(\b[11] ), .b(\a[12] ), .o1(new_n147));
  aob012aa1n02x5               g052(.a(new_n147), .b(new_n139), .c(new_n146), .out0(new_n148));
  oai022aa1n02x5               g053(.a(new_n148), .b(new_n145), .c(new_n144), .d(new_n141), .o1(\s[12] ));
  norp02aa1n03x5               g054(.a(\b[10] ), .b(\a[11] ), .o1(new_n150));
  aoi012aa1n02x7               g055(.a(new_n150), .b(\a[10] ), .c(\b[9] ), .o1(new_n151));
  oai022aa1n02x7               g056(.a(new_n142), .b(new_n143), .c(\b[11] ), .d(\a[12] ), .o1(new_n152));
  nanb03aa1n06x5               g057(.a(new_n152), .b(new_n147), .c(new_n151), .out0(new_n153));
  nano22aa1n03x7               g058(.a(new_n153), .b(new_n129), .c(new_n137), .out0(new_n154));
  aoai13aa1n06x5               g059(.a(new_n154), .b(new_n126), .c(new_n120), .d(new_n106), .o1(new_n155));
  nanp02aa1n02x5               g060(.a(new_n145), .b(new_n147), .o1(new_n156));
  oaih12aa1n12x5               g061(.a(new_n156), .b(new_n153), .c(new_n133), .o1(new_n157));
  inv000aa1d42x5               g062(.a(new_n157), .o1(new_n158));
  xorc02aa1n02x5               g063(.a(\a[13] ), .b(\b[12] ), .out0(new_n159));
  xnbna2aa1n03x5               g064(.a(new_n159), .b(new_n155), .c(new_n158), .out0(\s[13] ));
  nor042aa1n06x5               g065(.a(\b[12] ), .b(\a[13] ), .o1(new_n161));
  inv000aa1d42x5               g066(.a(new_n161), .o1(new_n162));
  nand22aa1n03x5               g067(.a(new_n155), .b(new_n158), .o1(new_n163));
  nanp02aa1n02x5               g068(.a(new_n163), .b(new_n159), .o1(new_n164));
  xorc02aa1n02x5               g069(.a(\a[14] ), .b(\b[13] ), .out0(new_n165));
  xnbna2aa1n03x5               g070(.a(new_n165), .b(new_n164), .c(new_n162), .out0(\s[14] ));
  inv000aa1d42x5               g071(.a(\a[13] ), .o1(new_n167));
  inv040aa1d32x5               g072(.a(\a[14] ), .o1(new_n168));
  xroi22aa1d04x5               g073(.a(new_n167), .b(\b[12] ), .c(new_n168), .d(\b[13] ), .out0(new_n169));
  inv000aa1d42x5               g074(.a(\b[13] ), .o1(new_n170));
  oaoi03aa1n12x5               g075(.a(new_n168), .b(new_n170), .c(new_n161), .o1(new_n171));
  inv000aa1d42x5               g076(.a(new_n171), .o1(new_n172));
  xorc02aa1n02x5               g077(.a(\a[15] ), .b(\b[14] ), .out0(new_n173));
  aoai13aa1n06x5               g078(.a(new_n173), .b(new_n172), .c(new_n163), .d(new_n169), .o1(new_n174));
  aoi112aa1n02x5               g079(.a(new_n173), .b(new_n172), .c(new_n163), .d(new_n169), .o1(new_n175));
  norb02aa1n02x5               g080(.a(new_n174), .b(new_n175), .out0(\s[15] ));
  inv000aa1d42x5               g081(.a(\a[15] ), .o1(new_n177));
  nanb02aa1n02x5               g082(.a(\b[14] ), .b(new_n177), .out0(new_n178));
  xorc02aa1n12x5               g083(.a(\a[16] ), .b(\b[15] ), .out0(new_n179));
  inv000aa1d42x5               g084(.a(new_n179), .o1(new_n180));
  tech160nm_fiaoi012aa1n04x5   g085(.a(new_n180), .b(new_n174), .c(new_n178), .o1(new_n181));
  nanp03aa1n02x5               g086(.a(new_n174), .b(new_n178), .c(new_n180), .o1(new_n182));
  norb02aa1n02x5               g087(.a(new_n182), .b(new_n181), .out0(\s[16] ));
  inv000aa1d42x5               g088(.a(\a[16] ), .o1(new_n184));
  xroi22aa1d04x5               g089(.a(new_n177), .b(\b[14] ), .c(new_n184), .d(\b[15] ), .out0(new_n185));
  nanp02aa1n02x5               g090(.a(new_n185), .b(new_n169), .o1(new_n186));
  nanp02aa1n02x5               g091(.a(new_n165), .b(new_n159), .o1(new_n187));
  nano22aa1n02x5               g092(.a(new_n187), .b(new_n173), .c(new_n179), .out0(new_n188));
  nand22aa1n03x5               g093(.a(new_n171), .b(new_n178), .o1(new_n189));
  aoi022aa1n02x5               g094(.a(\b[15] ), .b(\a[16] ), .c(\a[15] ), .d(\b[14] ), .o1(new_n190));
  nanp02aa1n02x5               g095(.a(new_n189), .b(new_n190), .o1(new_n191));
  oaib12aa1n02x5               g096(.a(new_n191), .b(\b[15] ), .c(new_n184), .out0(new_n192));
  tech160nm_fiaoi012aa1n04x5   g097(.a(new_n192), .b(new_n157), .c(new_n188), .o1(new_n193));
  oai012aa1n12x5               g098(.a(new_n193), .b(new_n155), .c(new_n186), .o1(new_n194));
  xorb03aa1n02x5               g099(.a(new_n194), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv000aa1d42x5               g100(.a(\a[17] ), .o1(new_n196));
  inv000aa1d42x5               g101(.a(\b[16] ), .o1(new_n197));
  nanp02aa1n02x5               g102(.a(new_n197), .b(new_n196), .o1(new_n198));
  nano23aa1n02x4               g103(.a(new_n152), .b(new_n150), .c(new_n136), .d(new_n147), .out0(new_n199));
  oai012aa1n02x5               g104(.a(new_n199), .b(new_n130), .c(new_n97), .o1(new_n200));
  inv000aa1d42x5               g105(.a(\b[15] ), .o1(new_n201));
  aoi022aa1n02x5               g106(.a(new_n189), .b(new_n190), .c(new_n201), .d(new_n184), .o1(new_n202));
  aoai13aa1n06x5               g107(.a(new_n202), .b(new_n186), .c(new_n200), .d(new_n156), .o1(new_n203));
  nano22aa1n03x7               g108(.a(new_n132), .b(new_n154), .c(new_n188), .out0(new_n204));
  oai022aa1n02x5               g109(.a(new_n204), .b(new_n203), .c(new_n197), .d(new_n196), .o1(new_n205));
  nor002aa1n02x5               g110(.a(\b[17] ), .b(\a[18] ), .o1(new_n206));
  nanp02aa1n02x5               g111(.a(\b[17] ), .b(\a[18] ), .o1(new_n207));
  norb02aa1n02x5               g112(.a(new_n207), .b(new_n206), .out0(new_n208));
  xnbna2aa1n03x5               g113(.a(new_n208), .b(new_n205), .c(new_n198), .out0(\s[18] ));
  nanp02aa1n02x5               g114(.a(\b[16] ), .b(\a[17] ), .o1(new_n210));
  nano32aa1n02x4               g115(.a(new_n206), .b(new_n198), .c(new_n207), .d(new_n210), .out0(new_n211));
  oaoi03aa1n06x5               g116(.a(\a[18] ), .b(\b[17] ), .c(new_n198), .o1(new_n212));
  nor042aa1n06x5               g117(.a(\b[18] ), .b(\a[19] ), .o1(new_n213));
  nand42aa1n04x5               g118(.a(\b[18] ), .b(\a[19] ), .o1(new_n214));
  norb02aa1n02x5               g119(.a(new_n214), .b(new_n213), .out0(new_n215));
  aoai13aa1n06x5               g120(.a(new_n215), .b(new_n212), .c(new_n194), .d(new_n211), .o1(new_n216));
  aoi112aa1n02x5               g121(.a(new_n215), .b(new_n212), .c(new_n194), .d(new_n211), .o1(new_n217));
  norb02aa1n02x5               g122(.a(new_n216), .b(new_n217), .out0(\s[19] ));
  xnrc02aa1n02x5               g123(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  inv040aa1n03x5               g124(.a(new_n213), .o1(new_n220));
  nor042aa1n03x5               g125(.a(\b[19] ), .b(\a[20] ), .o1(new_n221));
  nand42aa1n04x5               g126(.a(\b[19] ), .b(\a[20] ), .o1(new_n222));
  norb02aa1n02x5               g127(.a(new_n222), .b(new_n221), .out0(new_n223));
  norb03aa1n02x5               g128(.a(new_n222), .b(new_n213), .c(new_n221), .out0(new_n224));
  nand02aa1n03x5               g129(.a(new_n216), .b(new_n224), .o1(new_n225));
  aoai13aa1n03x5               g130(.a(new_n225), .b(new_n223), .c(new_n216), .d(new_n220), .o1(\s[20] ));
  nano23aa1n03x7               g131(.a(new_n213), .b(new_n221), .c(new_n222), .d(new_n214), .out0(new_n227));
  nanp02aa1n02x5               g132(.a(new_n211), .b(new_n227), .o1(new_n228));
  oabi12aa1n02x5               g133(.a(new_n228), .b(new_n204), .c(new_n203), .out0(new_n229));
  aoi013aa1n02x5               g134(.a(new_n206), .b(new_n207), .c(new_n196), .d(new_n197), .o1(new_n230));
  nona23aa1n09x5               g135(.a(new_n222), .b(new_n214), .c(new_n213), .d(new_n221), .out0(new_n231));
  oaoi03aa1n02x5               g136(.a(\a[20] ), .b(\b[19] ), .c(new_n220), .o1(new_n232));
  oabi12aa1n12x5               g137(.a(new_n232), .b(new_n231), .c(new_n230), .out0(new_n233));
  inv000aa1d42x5               g138(.a(new_n233), .o1(new_n234));
  xnrc02aa1n12x5               g139(.a(\b[20] ), .b(\a[21] ), .out0(new_n235));
  inv000aa1d42x5               g140(.a(new_n235), .o1(new_n236));
  xnbna2aa1n03x5               g141(.a(new_n236), .b(new_n229), .c(new_n234), .out0(\s[21] ));
  nona32aa1n02x4               g142(.a(new_n128), .b(new_n186), .c(new_n153), .d(new_n138), .out0(new_n238));
  aoai13aa1n02x5               g143(.a(new_n234), .b(new_n228), .c(new_n238), .d(new_n193), .o1(new_n239));
  nor042aa1n03x5               g144(.a(\b[20] ), .b(\a[21] ), .o1(new_n240));
  xnrc02aa1n03x5               g145(.a(\b[21] ), .b(\a[22] ), .out0(new_n241));
  aoai13aa1n02x5               g146(.a(new_n241), .b(new_n240), .c(new_n239), .d(new_n236), .o1(new_n242));
  norp02aa1n02x5               g147(.a(new_n241), .b(new_n240), .o1(new_n243));
  aoai13aa1n02x5               g148(.a(new_n243), .b(new_n235), .c(new_n229), .d(new_n234), .o1(new_n244));
  nanp02aa1n02x5               g149(.a(new_n242), .b(new_n244), .o1(\s[22] ));
  nor042aa1n06x5               g150(.a(new_n241), .b(new_n235), .o1(new_n246));
  and003aa1n02x5               g151(.a(new_n246), .b(new_n211), .c(new_n227), .o(new_n247));
  tech160nm_fioai012aa1n05x5   g152(.a(new_n247), .b(new_n204), .c(new_n203), .o1(new_n248));
  inv000aa1d42x5               g153(.a(\a[22] ), .o1(new_n249));
  inv000aa1d42x5               g154(.a(\b[21] ), .o1(new_n250));
  oao003aa1n02x5               g155(.a(new_n249), .b(new_n250), .c(new_n240), .carry(new_n251));
  aoi012aa1n06x5               g156(.a(new_n251), .b(new_n233), .c(new_n246), .o1(new_n252));
  xnrc02aa1n12x5               g157(.a(\b[22] ), .b(\a[23] ), .out0(new_n253));
  inv000aa1d42x5               g158(.a(new_n253), .o1(new_n254));
  xnbna2aa1n03x5               g159(.a(new_n254), .b(new_n248), .c(new_n252), .out0(\s[23] ));
  norp02aa1n02x5               g160(.a(\b[22] ), .b(\a[23] ), .o1(new_n256));
  inv000aa1d42x5               g161(.a(new_n256), .o1(new_n257));
  inv000aa1n02x5               g162(.a(new_n252), .o1(new_n258));
  aoai13aa1n02x5               g163(.a(new_n254), .b(new_n258), .c(new_n194), .d(new_n247), .o1(new_n259));
  xorc02aa1n02x5               g164(.a(\a[24] ), .b(\b[23] ), .out0(new_n260));
  nanp02aa1n02x5               g165(.a(\b[23] ), .b(\a[24] ), .o1(new_n261));
  oai022aa1n02x5               g166(.a(\a[23] ), .b(\b[22] ), .c(\b[23] ), .d(\a[24] ), .o1(new_n262));
  norb02aa1n02x5               g167(.a(new_n261), .b(new_n262), .out0(new_n263));
  aoai13aa1n03x5               g168(.a(new_n263), .b(new_n253), .c(new_n248), .d(new_n252), .o1(new_n264));
  aoai13aa1n02x5               g169(.a(new_n264), .b(new_n260), .c(new_n259), .d(new_n257), .o1(\s[24] ));
  nano32aa1n02x4               g170(.a(new_n228), .b(new_n260), .c(new_n246), .d(new_n254), .out0(new_n266));
  oai012aa1n02x5               g171(.a(new_n266), .b(new_n204), .c(new_n203), .o1(new_n267));
  aoai13aa1n06x5               g172(.a(new_n246), .b(new_n232), .c(new_n227), .d(new_n212), .o1(new_n268));
  inv000aa1n02x5               g173(.a(new_n251), .o1(new_n269));
  norb02aa1n06x5               g174(.a(new_n260), .b(new_n253), .out0(new_n270));
  inv000aa1d42x5               g175(.a(new_n270), .o1(new_n271));
  nanp02aa1n02x5               g176(.a(new_n262), .b(new_n261), .o1(new_n272));
  aoai13aa1n12x5               g177(.a(new_n272), .b(new_n271), .c(new_n268), .d(new_n269), .o1(new_n273));
  inv000aa1d42x5               g178(.a(new_n273), .o1(new_n274));
  tech160nm_fixorc02aa1n03p5x5 g179(.a(\a[25] ), .b(\b[24] ), .out0(new_n275));
  xnbna2aa1n03x5               g180(.a(new_n275), .b(new_n267), .c(new_n274), .out0(\s[25] ));
  norp02aa1n02x5               g181(.a(\b[24] ), .b(\a[25] ), .o1(new_n277));
  inv000aa1d42x5               g182(.a(new_n277), .o1(new_n278));
  aoai13aa1n03x5               g183(.a(new_n275), .b(new_n273), .c(new_n194), .d(new_n266), .o1(new_n279));
  xorc02aa1n02x5               g184(.a(\a[26] ), .b(\b[25] ), .out0(new_n280));
  nanp02aa1n02x5               g185(.a(\b[25] ), .b(\a[26] ), .o1(new_n281));
  oai022aa1n02x5               g186(.a(\a[25] ), .b(\b[24] ), .c(\b[25] ), .d(\a[26] ), .o1(new_n282));
  norb02aa1n02x5               g187(.a(new_n281), .b(new_n282), .out0(new_n283));
  nanp02aa1n03x5               g188(.a(new_n279), .b(new_n283), .o1(new_n284));
  aoai13aa1n02x5               g189(.a(new_n284), .b(new_n280), .c(new_n279), .d(new_n278), .o1(\s[26] ));
  nor042aa1n04x5               g190(.a(\b[26] ), .b(\a[27] ), .o1(new_n286));
  and002aa1n12x5               g191(.a(\b[26] ), .b(\a[27] ), .o(new_n287));
  norp02aa1n02x5               g192(.a(new_n287), .b(new_n286), .o1(new_n288));
  and002aa1n06x5               g193(.a(new_n280), .b(new_n275), .o(new_n289));
  nano32aa1n03x7               g194(.a(new_n228), .b(new_n289), .c(new_n246), .d(new_n270), .out0(new_n290));
  oai012aa1n04x7               g195(.a(new_n290), .b(new_n204), .c(new_n203), .o1(new_n291));
  aoi022aa1n12x5               g196(.a(new_n273), .b(new_n289), .c(new_n281), .d(new_n282), .o1(new_n292));
  xnbna2aa1n03x5               g197(.a(new_n288), .b(new_n292), .c(new_n291), .out0(\s[27] ));
  inv000aa1d42x5               g198(.a(new_n287), .o1(new_n294));
  aoai13aa1n06x5               g199(.a(new_n270), .b(new_n251), .c(new_n233), .d(new_n246), .o1(new_n295));
  inv000aa1d42x5               g200(.a(new_n289), .o1(new_n296));
  nanp02aa1n02x5               g201(.a(new_n282), .b(new_n281), .o1(new_n297));
  aoai13aa1n04x5               g202(.a(new_n297), .b(new_n296), .c(new_n295), .d(new_n272), .o1(new_n298));
  aoai13aa1n02x5               g203(.a(new_n294), .b(new_n298), .c(new_n290), .d(new_n194), .o1(new_n299));
  inv000aa1d42x5               g204(.a(new_n286), .o1(new_n300));
  aoai13aa1n03x5               g205(.a(new_n300), .b(new_n287), .c(new_n292), .d(new_n291), .o1(new_n301));
  xorc02aa1n02x5               g206(.a(\a[28] ), .b(\b[27] ), .out0(new_n302));
  norp02aa1n02x5               g207(.a(new_n302), .b(new_n286), .o1(new_n303));
  aoi022aa1n02x5               g208(.a(new_n301), .b(new_n302), .c(new_n299), .d(new_n303), .o1(\s[28] ));
  and002aa1n02x5               g209(.a(new_n302), .b(new_n288), .o(new_n305));
  aoai13aa1n02x5               g210(.a(new_n305), .b(new_n298), .c(new_n194), .d(new_n290), .o1(new_n306));
  inv000aa1n02x5               g211(.a(new_n305), .o1(new_n307));
  oao003aa1n02x5               g212(.a(\a[28] ), .b(\b[27] ), .c(new_n300), .carry(new_n308));
  aoai13aa1n03x5               g213(.a(new_n308), .b(new_n307), .c(new_n292), .d(new_n291), .o1(new_n309));
  xorc02aa1n02x5               g214(.a(\a[29] ), .b(\b[28] ), .out0(new_n310));
  norb02aa1n02x5               g215(.a(new_n308), .b(new_n310), .out0(new_n311));
  aoi022aa1n02x5               g216(.a(new_n309), .b(new_n310), .c(new_n306), .d(new_n311), .o1(\s[29] ));
  nanp02aa1n02x5               g217(.a(\b[0] ), .b(\a[1] ), .o1(new_n313));
  xorb03aa1n02x5               g218(.a(new_n313), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  and003aa1n02x5               g219(.a(new_n302), .b(new_n310), .c(new_n288), .o(new_n315));
  aoai13aa1n02x5               g220(.a(new_n315), .b(new_n298), .c(new_n194), .d(new_n290), .o1(new_n316));
  inv000aa1d42x5               g221(.a(new_n315), .o1(new_n317));
  oaoi03aa1n02x5               g222(.a(\a[29] ), .b(\b[28] ), .c(new_n308), .o1(new_n318));
  inv000aa1n03x5               g223(.a(new_n318), .o1(new_n319));
  aoai13aa1n03x5               g224(.a(new_n319), .b(new_n317), .c(new_n292), .d(new_n291), .o1(new_n320));
  xorc02aa1n02x5               g225(.a(\a[30] ), .b(\b[29] ), .out0(new_n321));
  and002aa1n02x5               g226(.a(\b[28] ), .b(\a[29] ), .o(new_n322));
  oabi12aa1n02x5               g227(.a(new_n321), .b(\a[29] ), .c(\b[28] ), .out0(new_n323));
  oab012aa1n02x4               g228(.a(new_n323), .b(new_n308), .c(new_n322), .out0(new_n324));
  aoi022aa1n03x5               g229(.a(new_n320), .b(new_n321), .c(new_n316), .d(new_n324), .o1(\s[30] ));
  nano22aa1n06x5               g230(.a(new_n307), .b(new_n310), .c(new_n321), .out0(new_n326));
  aoai13aa1n02x5               g231(.a(new_n326), .b(new_n298), .c(new_n194), .d(new_n290), .o1(new_n327));
  xorc02aa1n02x5               g232(.a(\a[31] ), .b(\b[30] ), .out0(new_n328));
  oao003aa1n02x5               g233(.a(\a[30] ), .b(\b[29] ), .c(new_n319), .carry(new_n329));
  norb02aa1n02x5               g234(.a(new_n329), .b(new_n328), .out0(new_n330));
  inv000aa1d42x5               g235(.a(new_n326), .o1(new_n331));
  aoai13aa1n03x5               g236(.a(new_n329), .b(new_n331), .c(new_n292), .d(new_n291), .o1(new_n332));
  aoi022aa1n02x5               g237(.a(new_n332), .b(new_n328), .c(new_n327), .d(new_n330), .o1(\s[31] ));
  xnbna2aa1n03x5               g238(.a(new_n102), .b(new_n104), .c(new_n103), .out0(\s[3] ));
  norb02aa1n02x5               g239(.a(new_n107), .b(new_n99), .out0(new_n335));
  aoi113aa1n02x5               g240(.a(new_n98), .b(new_n335), .c(new_n104), .d(new_n103), .e(new_n101), .o1(new_n336));
  aoi012aa1n02x5               g241(.a(new_n336), .b(new_n106), .c(new_n335), .o1(\s[4] ));
  nanb02aa1n02x5               g242(.a(new_n109), .b(new_n108), .out0(new_n338));
  xnbna2aa1n03x5               g243(.a(new_n338), .b(new_n106), .c(new_n107), .out0(\s[5] ));
  aoai13aa1n02x5               g244(.a(new_n108), .b(new_n109), .c(new_n106), .d(new_n107), .o1(new_n340));
  xnrb03aa1n02x5               g245(.a(new_n340), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  oaoi03aa1n02x5               g246(.a(\a[6] ), .b(\b[5] ), .c(new_n340), .o1(new_n342));
  xorb03aa1n02x5               g247(.a(new_n342), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  tech160nm_fiaoi012aa1n05x5   g248(.a(new_n117), .b(new_n342), .c(new_n118), .o1(new_n344));
  xnbna2aa1n03x5               g249(.a(new_n344), .b(new_n110), .c(new_n122), .out0(\s[8] ));
  xnbna2aa1n03x5               g250(.a(new_n129), .b(new_n121), .c(new_n127), .out0(\s[9] ));
endmodule


