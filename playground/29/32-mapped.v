// Benchmark "adder" written by ABC on Thu Jul 18 03:06:26 2024

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
    new_n140, new_n141, new_n142, new_n144, new_n145, new_n146, new_n147,
    new_n148, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n159, new_n160, new_n161, new_n162,
    new_n164, new_n166, new_n167, new_n168, new_n169, new_n170, new_n171,
    new_n172, new_n173, new_n174, new_n175, new_n177, new_n178, new_n179,
    new_n180, new_n181, new_n182, new_n183, new_n185, new_n186, new_n187,
    new_n188, new_n189, new_n190, new_n191, new_n192, new_n193, new_n194,
    new_n195, new_n196, new_n197, new_n198, new_n199, new_n201, new_n203,
    new_n204, new_n205, new_n206, new_n207, new_n208, new_n209, new_n210,
    new_n211, new_n212, new_n213, new_n214, new_n217, new_n218, new_n219,
    new_n220, new_n221, new_n222, new_n223, new_n225, new_n226, new_n227,
    new_n228, new_n229, new_n230, new_n231, new_n232, new_n233, new_n234,
    new_n235, new_n236, new_n237, new_n238, new_n240, new_n241, new_n242,
    new_n243, new_n245, new_n246, new_n247, new_n248, new_n249, new_n250,
    new_n251, new_n252, new_n253, new_n254, new_n255, new_n257, new_n258,
    new_n259, new_n260, new_n261, new_n263, new_n264, new_n265, new_n266,
    new_n267, new_n268, new_n269, new_n270, new_n271, new_n272, new_n273,
    new_n274, new_n275, new_n276, new_n277, new_n278, new_n279, new_n280,
    new_n282, new_n283, new_n284, new_n285, new_n286, new_n287, new_n288,
    new_n290, new_n291, new_n292, new_n293, new_n294, new_n295, new_n296,
    new_n297, new_n298, new_n300, new_n301, new_n302, new_n303, new_n304,
    new_n305, new_n306, new_n307, new_n308, new_n309, new_n311, new_n312,
    new_n313, new_n314, new_n315, new_n316, new_n317, new_n318, new_n319,
    new_n322, new_n323, new_n324, new_n325, new_n326, new_n327, new_n328,
    new_n330, new_n331, new_n332, new_n333, new_n334, new_n335, new_n336,
    new_n337, new_n338, new_n340, new_n342, new_n343, new_n345, new_n347,
    new_n349, new_n350, new_n352, new_n354;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor002aa1d32x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  inv000aa1d42x5               g002(.a(\a[2] ), .o1(new_n98));
  inv000aa1d42x5               g003(.a(\b[1] ), .o1(new_n99));
  nand02aa1n02x5               g004(.a(\b[0] ), .b(\a[1] ), .o1(new_n100));
  oao003aa1n02x5               g005(.a(new_n98), .b(new_n99), .c(new_n100), .carry(new_n101));
  nand42aa1n04x5               g006(.a(\b[3] ), .b(\a[4] ), .o1(new_n102));
  norp02aa1n04x5               g007(.a(\b[3] ), .b(\a[4] ), .o1(new_n103));
  nor042aa1n06x5               g008(.a(\b[2] ), .b(\a[3] ), .o1(new_n104));
  nand42aa1n03x5               g009(.a(\b[2] ), .b(\a[3] ), .o1(new_n105));
  nano23aa1n02x4               g010(.a(new_n104), .b(new_n103), .c(new_n105), .d(new_n102), .out0(new_n106));
  nanp02aa1n02x5               g011(.a(new_n106), .b(new_n101), .o1(new_n107));
  tech160nm_fiaoi012aa1n03p5x5 g012(.a(new_n103), .b(new_n104), .c(new_n102), .o1(new_n108));
  nor042aa1n02x5               g013(.a(\b[5] ), .b(\a[6] ), .o1(new_n109));
  nanp02aa1n04x5               g014(.a(\b[5] ), .b(\a[6] ), .o1(new_n110));
  nor002aa1n06x5               g015(.a(\b[4] ), .b(\a[5] ), .o1(new_n111));
  nanp02aa1n02x5               g016(.a(\b[4] ), .b(\a[5] ), .o1(new_n112));
  nano23aa1n03x7               g017(.a(new_n109), .b(new_n111), .c(new_n112), .d(new_n110), .out0(new_n113));
  nor002aa1n02x5               g018(.a(\b[7] ), .b(\a[8] ), .o1(new_n114));
  nand42aa1n02x5               g019(.a(\b[7] ), .b(\a[8] ), .o1(new_n115));
  nanb02aa1n03x5               g020(.a(new_n114), .b(new_n115), .out0(new_n116));
  nand42aa1n02x5               g021(.a(\b[6] ), .b(\a[7] ), .o1(new_n117));
  nor002aa1n03x5               g022(.a(\b[6] ), .b(\a[7] ), .o1(new_n118));
  nanb02aa1n03x5               g023(.a(new_n118), .b(new_n117), .out0(new_n119));
  nona22aa1n03x5               g024(.a(new_n113), .b(new_n116), .c(new_n119), .out0(new_n120));
  nona22aa1n06x5               g025(.a(new_n115), .b(new_n118), .c(new_n114), .out0(new_n121));
  aoai13aa1n06x5               g026(.a(new_n117), .b(new_n109), .c(new_n111), .d(new_n110), .o1(new_n122));
  oa0012aa1n06x5               g027(.a(new_n115), .b(new_n118), .c(new_n114), .o(new_n123));
  oab012aa1n06x5               g028(.a(new_n123), .b(new_n122), .c(new_n121), .out0(new_n124));
  aoai13aa1n12x5               g029(.a(new_n124), .b(new_n120), .c(new_n107), .d(new_n108), .o1(new_n125));
  nanp02aa1n04x5               g030(.a(\b[8] ), .b(\a[9] ), .o1(new_n126));
  aoi012aa1n02x5               g031(.a(new_n97), .b(new_n125), .c(new_n126), .o1(new_n127));
  xnrb03aa1n02x5               g032(.a(new_n127), .b(\b[9] ), .c(\a[10] ), .out0(\s[10] ));
  oaoi03aa1n02x5               g033(.a(new_n98), .b(new_n99), .c(new_n100), .o1(new_n129));
  nona23aa1n02x4               g034(.a(new_n102), .b(new_n105), .c(new_n104), .d(new_n103), .out0(new_n130));
  oai012aa1n06x5               g035(.a(new_n108), .b(new_n130), .c(new_n129), .o1(new_n131));
  nona23aa1n02x4               g036(.a(new_n112), .b(new_n110), .c(new_n109), .d(new_n111), .out0(new_n132));
  nor043aa1n02x5               g037(.a(new_n132), .b(new_n116), .c(new_n119), .o1(new_n133));
  oabi12aa1n02x5               g038(.a(new_n123), .b(new_n121), .c(new_n122), .out0(new_n134));
  nor002aa1n16x5               g039(.a(\b[9] ), .b(\a[10] ), .o1(new_n135));
  nand22aa1n12x5               g040(.a(\b[9] ), .b(\a[10] ), .o1(new_n136));
  nano23aa1n09x5               g041(.a(new_n135), .b(new_n97), .c(new_n126), .d(new_n136), .out0(new_n137));
  aoai13aa1n02x5               g042(.a(new_n137), .b(new_n134), .c(new_n131), .d(new_n133), .o1(new_n138));
  aoi012aa1n02x5               g043(.a(new_n135), .b(new_n97), .c(new_n136), .o1(new_n139));
  nor002aa1n16x5               g044(.a(\b[10] ), .b(\a[11] ), .o1(new_n140));
  nand42aa1n20x5               g045(.a(\b[10] ), .b(\a[11] ), .o1(new_n141));
  norb02aa1n02x5               g046(.a(new_n141), .b(new_n140), .out0(new_n142));
  xnbna2aa1n03x5               g047(.a(new_n142), .b(new_n138), .c(new_n139), .out0(\s[11] ));
  inv000aa1d42x5               g048(.a(new_n140), .o1(new_n144));
  aob012aa1n02x5               g049(.a(new_n142), .b(new_n138), .c(new_n139), .out0(new_n145));
  nor002aa1d32x5               g050(.a(\b[11] ), .b(\a[12] ), .o1(new_n146));
  nand42aa1n06x5               g051(.a(\b[11] ), .b(\a[12] ), .o1(new_n147));
  nanb02aa1n02x5               g052(.a(new_n146), .b(new_n147), .out0(new_n148));
  xobna2aa1n03x5               g053(.a(new_n148), .b(new_n145), .c(new_n144), .out0(\s[12] ));
  nanp02aa1n03x5               g054(.a(new_n131), .b(new_n133), .o1(new_n150));
  nano23aa1n03x5               g055(.a(new_n140), .b(new_n146), .c(new_n147), .d(new_n141), .out0(new_n151));
  nand02aa1n02x5               g056(.a(new_n151), .b(new_n137), .o1(new_n152));
  inv000aa1d42x5               g057(.a(new_n146), .o1(new_n153));
  inv030aa1n02x5               g058(.a(new_n147), .o1(new_n154));
  aoai13aa1n12x5               g059(.a(new_n141), .b(new_n135), .c(new_n97), .d(new_n136), .o1(new_n155));
  aoai13aa1n12x5               g060(.a(new_n153), .b(new_n154), .c(new_n155), .d(new_n144), .o1(new_n156));
  inv000aa1d42x5               g061(.a(new_n156), .o1(new_n157));
  aoai13aa1n06x5               g062(.a(new_n157), .b(new_n152), .c(new_n150), .d(new_n124), .o1(new_n158));
  nor042aa1n06x5               g063(.a(\b[12] ), .b(\a[13] ), .o1(new_n159));
  nand02aa1n03x5               g064(.a(\b[12] ), .b(\a[13] ), .o1(new_n160));
  norb02aa1n02x5               g065(.a(new_n160), .b(new_n159), .out0(new_n161));
  aoi113aa1n02x5               g066(.a(new_n156), .b(new_n161), .c(new_n125), .d(new_n137), .e(new_n151), .o1(new_n162));
  aoi012aa1n02x5               g067(.a(new_n162), .b(new_n158), .c(new_n161), .o1(\s[13] ));
  aoi012aa1n02x5               g068(.a(new_n159), .b(new_n158), .c(new_n160), .o1(new_n164));
  xnrb03aa1n03x5               g069(.a(new_n164), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  norp02aa1n04x5               g070(.a(\b[13] ), .b(\a[14] ), .o1(new_n166));
  nanp02aa1n04x5               g071(.a(\b[13] ), .b(\a[14] ), .o1(new_n167));
  nano23aa1n06x5               g072(.a(new_n159), .b(new_n166), .c(new_n167), .d(new_n160), .out0(new_n168));
  aoi012aa1n02x5               g073(.a(new_n166), .b(new_n159), .c(new_n167), .o1(new_n169));
  inv000aa1d42x5               g074(.a(new_n169), .o1(new_n170));
  nor002aa1n16x5               g075(.a(\b[14] ), .b(\a[15] ), .o1(new_n171));
  nand02aa1n06x5               g076(.a(\b[14] ), .b(\a[15] ), .o1(new_n172));
  norb02aa1n03x5               g077(.a(new_n172), .b(new_n171), .out0(new_n173));
  aoai13aa1n06x5               g078(.a(new_n173), .b(new_n170), .c(new_n158), .d(new_n168), .o1(new_n174));
  aoi112aa1n02x5               g079(.a(new_n173), .b(new_n170), .c(new_n158), .d(new_n168), .o1(new_n175));
  norb02aa1n02x5               g080(.a(new_n174), .b(new_n175), .out0(\s[15] ));
  nor002aa1n02x5               g081(.a(\b[15] ), .b(\a[16] ), .o1(new_n177));
  nand42aa1n16x5               g082(.a(\b[15] ), .b(\a[16] ), .o1(new_n178));
  nanb02aa1n02x5               g083(.a(new_n177), .b(new_n178), .out0(new_n179));
  inv000aa1d42x5               g084(.a(new_n171), .o1(new_n180));
  nanp02aa1n03x5               g085(.a(new_n174), .b(new_n180), .o1(new_n181));
  nanp02aa1n02x5               g086(.a(new_n181), .b(new_n179), .o1(new_n182));
  nona22aa1n02x4               g087(.a(new_n174), .b(new_n179), .c(new_n171), .out0(new_n183));
  nanp02aa1n02x5               g088(.a(new_n182), .b(new_n183), .o1(\s[16] ));
  nano23aa1n02x5               g089(.a(new_n171), .b(new_n177), .c(new_n178), .d(new_n172), .out0(new_n185));
  nano22aa1n06x5               g090(.a(new_n152), .b(new_n168), .c(new_n185), .out0(new_n186));
  aoai13aa1n06x5               g091(.a(new_n186), .b(new_n134), .c(new_n131), .d(new_n133), .o1(new_n187));
  nona23aa1n03x5               g092(.a(new_n167), .b(new_n160), .c(new_n159), .d(new_n166), .out0(new_n188));
  inv000aa1n02x5               g093(.a(new_n177), .o1(new_n189));
  nano32aa1n06x5               g094(.a(new_n188), .b(new_n178), .c(new_n173), .d(new_n189), .out0(new_n190));
  inv000aa1d42x5               g095(.a(new_n178), .o1(new_n191));
  aoai13aa1n06x5               g096(.a(new_n172), .b(new_n166), .c(new_n159), .d(new_n167), .o1(new_n192));
  aoai13aa1n04x5               g097(.a(new_n189), .b(new_n191), .c(new_n192), .d(new_n180), .o1(new_n193));
  aoi012aa1n09x5               g098(.a(new_n193), .b(new_n156), .c(new_n190), .o1(new_n194));
  nanp02aa1n06x5               g099(.a(new_n187), .b(new_n194), .o1(new_n195));
  nor042aa1n04x5               g100(.a(\b[16] ), .b(\a[17] ), .o1(new_n196));
  nand42aa1n04x5               g101(.a(\b[16] ), .b(\a[17] ), .o1(new_n197));
  norb02aa1n02x5               g102(.a(new_n197), .b(new_n196), .out0(new_n198));
  aoi112aa1n02x5               g103(.a(new_n198), .b(new_n193), .c(new_n156), .d(new_n190), .o1(new_n199));
  aoi022aa1n02x5               g104(.a(new_n195), .b(new_n198), .c(new_n187), .d(new_n199), .o1(\s[17] ));
  tech160nm_fiaoi012aa1n05x5   g105(.a(new_n196), .b(new_n195), .c(new_n198), .o1(new_n201));
  xnrb03aa1n03x5               g106(.a(new_n201), .b(\b[17] ), .c(\a[18] ), .out0(\s[18] ));
  nanp02aa1n06x5               g107(.a(new_n156), .b(new_n190), .o1(new_n203));
  inv000aa1n02x5               g108(.a(new_n193), .o1(new_n204));
  nanp02aa1n09x5               g109(.a(new_n203), .b(new_n204), .o1(new_n205));
  nor042aa1n03x5               g110(.a(\b[17] ), .b(\a[18] ), .o1(new_n206));
  nand42aa1n06x5               g111(.a(\b[17] ), .b(\a[18] ), .o1(new_n207));
  nano23aa1n09x5               g112(.a(new_n196), .b(new_n206), .c(new_n207), .d(new_n197), .out0(new_n208));
  aoai13aa1n06x5               g113(.a(new_n208), .b(new_n205), .c(new_n125), .d(new_n186), .o1(new_n209));
  oa0012aa1n02x5               g114(.a(new_n207), .b(new_n206), .c(new_n196), .o(new_n210));
  inv000aa1d42x5               g115(.a(new_n210), .o1(new_n211));
  nor002aa1n16x5               g116(.a(\b[18] ), .b(\a[19] ), .o1(new_n212));
  nand02aa1n06x5               g117(.a(\b[18] ), .b(\a[19] ), .o1(new_n213));
  norb02aa1n06x5               g118(.a(new_n213), .b(new_n212), .out0(new_n214));
  xnbna2aa1n03x5               g119(.a(new_n214), .b(new_n209), .c(new_n211), .out0(\s[19] ));
  xnrc02aa1n02x5               g120(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nand22aa1n02x5               g121(.a(new_n209), .b(new_n211), .o1(new_n217));
  nor042aa1n09x5               g122(.a(\b[19] ), .b(\a[20] ), .o1(new_n218));
  nand22aa1n09x5               g123(.a(\b[19] ), .b(\a[20] ), .o1(new_n219));
  nanb02aa1n02x5               g124(.a(new_n218), .b(new_n219), .out0(new_n220));
  aoai13aa1n02x5               g125(.a(new_n220), .b(new_n212), .c(new_n217), .d(new_n213), .o1(new_n221));
  aoai13aa1n06x5               g126(.a(new_n214), .b(new_n210), .c(new_n195), .d(new_n208), .o1(new_n222));
  nona22aa1n03x5               g127(.a(new_n222), .b(new_n220), .c(new_n212), .out0(new_n223));
  nanp02aa1n03x5               g128(.a(new_n221), .b(new_n223), .o1(\s[20] ));
  nanb03aa1n06x5               g129(.a(new_n218), .b(new_n219), .c(new_n213), .out0(new_n225));
  oaih22aa1d12x5               g130(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n226));
  inv000aa1d42x5               g131(.a(\b[18] ), .o1(new_n227));
  nanb02aa1n02x5               g132(.a(\a[19] ), .b(new_n227), .out0(new_n228));
  nanp03aa1n03x5               g133(.a(new_n226), .b(new_n228), .c(new_n207), .o1(new_n229));
  tech160nm_fiaoi012aa1n03p5x5 g134(.a(new_n218), .b(new_n212), .c(new_n219), .o1(new_n230));
  oaih12aa1n12x5               g135(.a(new_n230), .b(new_n229), .c(new_n225), .o1(new_n231));
  inv000aa1d42x5               g136(.a(new_n231), .o1(new_n232));
  nanb03aa1n09x5               g137(.a(new_n220), .b(new_n208), .c(new_n214), .out0(new_n233));
  aoai13aa1n06x5               g138(.a(new_n232), .b(new_n233), .c(new_n187), .d(new_n194), .o1(new_n234));
  xnrc02aa1n12x5               g139(.a(\b[20] ), .b(\a[21] ), .out0(new_n235));
  inv000aa1d42x5               g140(.a(new_n235), .o1(new_n236));
  inv040aa1n02x5               g141(.a(new_n233), .o1(new_n237));
  aoi012aa1n02x5               g142(.a(new_n236), .b(new_n195), .c(new_n237), .o1(new_n238));
  aoi022aa1n02x5               g143(.a(new_n238), .b(new_n232), .c(new_n234), .d(new_n236), .o1(\s[21] ));
  nor042aa1n03x5               g144(.a(\b[20] ), .b(\a[21] ), .o1(new_n240));
  xnrc02aa1n12x5               g145(.a(\b[21] ), .b(\a[22] ), .out0(new_n241));
  aoai13aa1n04x5               g146(.a(new_n241), .b(new_n240), .c(new_n234), .d(new_n236), .o1(new_n242));
  aoi112aa1n03x5               g147(.a(new_n240), .b(new_n241), .c(new_n234), .d(new_n236), .o1(new_n243));
  nanb02aa1n03x5               g148(.a(new_n243), .b(new_n242), .out0(\s[22] ));
  nor042aa1d18x5               g149(.a(new_n241), .b(new_n235), .o1(new_n245));
  inv000aa1d42x5               g150(.a(new_n245), .o1(new_n246));
  norp02aa1n02x5               g151(.a(new_n233), .b(new_n246), .o1(new_n247));
  aoai13aa1n06x5               g152(.a(new_n247), .b(new_n205), .c(new_n125), .d(new_n186), .o1(new_n248));
  inv000aa1d42x5               g153(.a(\a[22] ), .o1(new_n249));
  inv000aa1d42x5               g154(.a(\b[21] ), .o1(new_n250));
  oao003aa1n02x5               g155(.a(new_n249), .b(new_n250), .c(new_n240), .carry(new_n251));
  aoi012aa1n02x5               g156(.a(new_n251), .b(new_n231), .c(new_n245), .o1(new_n252));
  nand02aa1d04x5               g157(.a(new_n248), .b(new_n252), .o1(new_n253));
  xorc02aa1n12x5               g158(.a(\a[23] ), .b(\b[22] ), .out0(new_n254));
  aoi112aa1n02x5               g159(.a(new_n254), .b(new_n251), .c(new_n231), .d(new_n245), .o1(new_n255));
  aoi022aa1n02x5               g160(.a(new_n253), .b(new_n254), .c(new_n248), .d(new_n255), .o1(\s[23] ));
  norp02aa1n02x5               g161(.a(\b[22] ), .b(\a[23] ), .o1(new_n257));
  xnrc02aa1n02x5               g162(.a(\b[23] ), .b(\a[24] ), .out0(new_n258));
  aoai13aa1n03x5               g163(.a(new_n258), .b(new_n257), .c(new_n253), .d(new_n254), .o1(new_n259));
  nand22aa1n02x5               g164(.a(new_n253), .b(new_n254), .o1(new_n260));
  nona22aa1n02x4               g165(.a(new_n260), .b(new_n258), .c(new_n257), .out0(new_n261));
  nanp02aa1n03x5               g166(.a(new_n261), .b(new_n259), .o1(\s[24] ));
  nanb02aa1d30x5               g167(.a(new_n258), .b(new_n254), .out0(new_n263));
  nor043aa1n02x5               g168(.a(new_n233), .b(new_n246), .c(new_n263), .o1(new_n264));
  aoai13aa1n06x5               g169(.a(new_n264), .b(new_n205), .c(new_n125), .d(new_n186), .o1(new_n265));
  nano22aa1n02x5               g170(.a(new_n218), .b(new_n213), .c(new_n219), .out0(new_n266));
  oai012aa1n02x5               g171(.a(new_n207), .b(\b[18] ), .c(\a[19] ), .o1(new_n267));
  oab012aa1n06x5               g172(.a(new_n267), .b(new_n196), .c(new_n206), .out0(new_n268));
  inv040aa1n03x5               g173(.a(new_n230), .o1(new_n269));
  aoai13aa1n06x5               g174(.a(new_n245), .b(new_n269), .c(new_n268), .d(new_n266), .o1(new_n270));
  inv000aa1n02x5               g175(.a(new_n251), .o1(new_n271));
  oai022aa1n02x5               g176(.a(\a[23] ), .b(\b[22] ), .c(\b[23] ), .d(\a[24] ), .o1(new_n272));
  aob012aa1n02x5               g177(.a(new_n272), .b(\b[23] ), .c(\a[24] ), .out0(new_n273));
  aoai13aa1n06x5               g178(.a(new_n273), .b(new_n263), .c(new_n270), .d(new_n271), .o1(new_n274));
  inv000aa1n02x5               g179(.a(new_n274), .o1(new_n275));
  nanp02aa1n06x5               g180(.a(new_n265), .b(new_n275), .o1(new_n276));
  xorc02aa1n12x5               g181(.a(\a[25] ), .b(\b[24] ), .out0(new_n277));
  inv000aa1n02x5               g182(.a(new_n263), .o1(new_n278));
  aoai13aa1n06x5               g183(.a(new_n278), .b(new_n251), .c(new_n231), .d(new_n245), .o1(new_n279));
  nano22aa1n02x4               g184(.a(new_n277), .b(new_n279), .c(new_n273), .out0(new_n280));
  aoi022aa1n02x5               g185(.a(new_n276), .b(new_n277), .c(new_n265), .d(new_n280), .o1(\s[25] ));
  norp02aa1n02x5               g186(.a(\b[24] ), .b(\a[25] ), .o1(new_n282));
  nor022aa1n04x5               g187(.a(\b[25] ), .b(\a[26] ), .o1(new_n283));
  nand42aa1n03x5               g188(.a(\b[25] ), .b(\a[26] ), .o1(new_n284));
  nanb02aa1n09x5               g189(.a(new_n283), .b(new_n284), .out0(new_n285));
  aoai13aa1n02x7               g190(.a(new_n285), .b(new_n282), .c(new_n276), .d(new_n277), .o1(new_n286));
  aoai13aa1n03x5               g191(.a(new_n277), .b(new_n274), .c(new_n195), .d(new_n264), .o1(new_n287));
  nona22aa1n03x5               g192(.a(new_n287), .b(new_n285), .c(new_n282), .out0(new_n288));
  nanp02aa1n03x5               g193(.a(new_n286), .b(new_n288), .o1(\s[26] ));
  norb02aa1n06x5               g194(.a(new_n277), .b(new_n285), .out0(new_n290));
  nano23aa1n03x7               g195(.a(new_n233), .b(new_n263), .c(new_n290), .d(new_n245), .out0(new_n291));
  aoai13aa1n06x5               g196(.a(new_n291), .b(new_n205), .c(new_n125), .d(new_n186), .o1(new_n292));
  oai012aa1n02x5               g197(.a(new_n284), .b(new_n283), .c(new_n282), .o1(new_n293));
  aobi12aa1n06x5               g198(.a(new_n293), .b(new_n274), .c(new_n290), .out0(new_n294));
  nanp02aa1n03x5               g199(.a(new_n294), .b(new_n292), .o1(new_n295));
  xorc02aa1n02x5               g200(.a(\a[27] ), .b(\b[26] ), .out0(new_n296));
  nanb02aa1n02x5               g201(.a(new_n296), .b(new_n293), .out0(new_n297));
  aoi012aa1n02x5               g202(.a(new_n297), .b(new_n274), .c(new_n290), .o1(new_n298));
  aoi022aa1n02x5               g203(.a(new_n295), .b(new_n296), .c(new_n292), .d(new_n298), .o1(\s[27] ));
  norp02aa1n02x5               g204(.a(\b[26] ), .b(\a[27] ), .o1(new_n300));
  xorc02aa1n12x5               g205(.a(\a[28] ), .b(\b[27] ), .out0(new_n301));
  inv000aa1d42x5               g206(.a(new_n301), .o1(new_n302));
  aoai13aa1n03x5               g207(.a(new_n302), .b(new_n300), .c(new_n295), .d(new_n296), .o1(new_n303));
  nona23aa1n08x5               g208(.a(new_n237), .b(new_n290), .c(new_n263), .d(new_n246), .out0(new_n304));
  aoi012aa1n06x5               g209(.a(new_n304), .b(new_n187), .c(new_n194), .o1(new_n305));
  inv000aa1d42x5               g210(.a(new_n290), .o1(new_n306));
  aoai13aa1n06x5               g211(.a(new_n293), .b(new_n306), .c(new_n279), .d(new_n273), .o1(new_n307));
  oaih12aa1n02x5               g212(.a(new_n296), .b(new_n307), .c(new_n305), .o1(new_n308));
  nona22aa1n02x5               g213(.a(new_n308), .b(new_n302), .c(new_n300), .out0(new_n309));
  nanp02aa1n03x5               g214(.a(new_n303), .b(new_n309), .o1(\s[28] ));
  xnrc02aa1n02x5               g215(.a(\b[28] ), .b(\a[29] ), .out0(new_n311));
  and002aa1n24x5               g216(.a(new_n301), .b(new_n296), .o(new_n312));
  oaih12aa1n02x5               g217(.a(new_n312), .b(new_n307), .c(new_n305), .o1(new_n313));
  aoi112aa1n09x5               g218(.a(\b[26] ), .b(\a[27] ), .c(\a[28] ), .d(\b[27] ), .o1(new_n314));
  oab012aa1n06x5               g219(.a(new_n314), .b(\a[28] ), .c(\b[27] ), .out0(new_n315));
  aoi012aa1n03x5               g220(.a(new_n311), .b(new_n313), .c(new_n315), .o1(new_n316));
  inv000aa1d42x5               g221(.a(new_n312), .o1(new_n317));
  tech160nm_fiaoi012aa1n05x5   g222(.a(new_n317), .b(new_n294), .c(new_n292), .o1(new_n318));
  nano22aa1n03x7               g223(.a(new_n318), .b(new_n311), .c(new_n315), .out0(new_n319));
  nor002aa1n02x5               g224(.a(new_n316), .b(new_n319), .o1(\s[29] ));
  xorb03aa1n02x5               g225(.a(new_n100), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n02x4               g226(.a(new_n311), .b(new_n296), .c(new_n301), .out0(new_n322));
  oaih12aa1n02x5               g227(.a(new_n322), .b(new_n307), .c(new_n305), .o1(new_n323));
  tech160nm_fioaoi03aa1n02p5x5 g228(.a(\a[29] ), .b(\b[28] ), .c(new_n315), .o1(new_n324));
  inv000aa1d42x5               g229(.a(new_n324), .o1(new_n325));
  nanp02aa1n03x5               g230(.a(new_n323), .b(new_n325), .o1(new_n326));
  xorc02aa1n02x5               g231(.a(\a[30] ), .b(\b[29] ), .out0(new_n327));
  norp02aa1n02x5               g232(.a(new_n324), .b(new_n327), .o1(new_n328));
  aoi022aa1n02x7               g233(.a(new_n326), .b(new_n327), .c(new_n323), .d(new_n328), .o1(\s[30] ));
  nano32aa1n02x4               g234(.a(new_n311), .b(new_n327), .c(new_n296), .d(new_n301), .out0(new_n330));
  oaih12aa1n02x5               g235(.a(new_n330), .b(new_n307), .c(new_n305), .o1(new_n331));
  xorc02aa1n02x5               g236(.a(\a[31] ), .b(\b[30] ), .out0(new_n332));
  inv000aa1d42x5               g237(.a(\a[30] ), .o1(new_n333));
  inv000aa1d42x5               g238(.a(\b[29] ), .o1(new_n334));
  oabi12aa1n02x5               g239(.a(new_n332), .b(\a[30] ), .c(\b[29] ), .out0(new_n335));
  oaoi13aa1n04x5               g240(.a(new_n335), .b(new_n324), .c(new_n333), .d(new_n334), .o1(new_n336));
  oaoi03aa1n03x5               g241(.a(new_n333), .b(new_n334), .c(new_n324), .o1(new_n337));
  nanp02aa1n03x5               g242(.a(new_n331), .b(new_n337), .o1(new_n338));
  aoi022aa1n02x7               g243(.a(new_n338), .b(new_n332), .c(new_n331), .d(new_n336), .o1(\s[31] ));
  norb02aa1n02x5               g244(.a(new_n105), .b(new_n104), .out0(new_n340));
  xnrc02aa1n02x5               g245(.a(new_n129), .b(new_n340), .out0(\s[3] ));
  obai22aa1n02x7               g246(.a(new_n102), .b(new_n103), .c(\a[3] ), .d(\b[2] ), .out0(new_n342));
  aoi012aa1n02x5               g247(.a(new_n342), .b(new_n101), .c(new_n340), .o1(new_n343));
  oaoi13aa1n02x5               g248(.a(new_n343), .b(new_n131), .c(\a[4] ), .d(\b[3] ), .o1(\s[4] ));
  nanb02aa1n02x5               g249(.a(new_n111), .b(new_n112), .out0(new_n345));
  xobna2aa1n03x5               g250(.a(new_n345), .b(new_n107), .c(new_n108), .out0(\s[5] ));
  aoi012aa1n02x5               g251(.a(new_n111), .b(new_n131), .c(new_n112), .o1(new_n347));
  xnrb03aa1n02x5               g252(.a(new_n347), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  tech160nm_fiao0012aa1n02p5x5 g253(.a(new_n109), .b(new_n111), .c(new_n110), .o(new_n349));
  tech160nm_fiao0012aa1n02p5x5 g254(.a(new_n349), .b(new_n131), .c(new_n113), .o(new_n350));
  xorb03aa1n02x5               g255(.a(new_n350), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  aoi012aa1n02x5               g256(.a(new_n118), .b(new_n350), .c(new_n117), .o1(new_n352));
  xnrb03aa1n02x5               g257(.a(new_n352), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  norb02aa1n02x5               g258(.a(new_n126), .b(new_n97), .out0(new_n354));
  xnbna2aa1n03x5               g259(.a(new_n354), .b(new_n150), .c(new_n124), .out0(\s[9] ));
endmodule


