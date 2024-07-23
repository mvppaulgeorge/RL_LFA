// Benchmark "adder" written by ABC on Wed Jul 17 22:25:23 2024

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
    new_n125, new_n127, new_n128, new_n129, new_n130, new_n131, new_n133,
    new_n134, new_n135, new_n136, new_n138, new_n139, new_n140, new_n141,
    new_n142, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n150, new_n151, new_n153, new_n154, new_n155, new_n156, new_n157,
    new_n159, new_n160, new_n161, new_n162, new_n163, new_n165, new_n166,
    new_n167, new_n168, new_n169, new_n170, new_n171, new_n172, new_n173,
    new_n174, new_n176, new_n177, new_n178, new_n179, new_n180, new_n181,
    new_n182, new_n183, new_n184, new_n185, new_n186, new_n187, new_n188,
    new_n190, new_n191, new_n192, new_n193, new_n194, new_n195, new_n196,
    new_n197, new_n198, new_n201, new_n202, new_n203, new_n204, new_n205,
    new_n206, new_n207, new_n209, new_n210, new_n211, new_n212, new_n213,
    new_n214, new_n215, new_n216, new_n217, new_n218, new_n219, new_n220,
    new_n222, new_n223, new_n224, new_n225, new_n226, new_n227, new_n229,
    new_n230, new_n231, new_n232, new_n233, new_n234, new_n235, new_n236,
    new_n237, new_n238, new_n240, new_n241, new_n242, new_n243, new_n244,
    new_n245, new_n246, new_n248, new_n249, new_n250, new_n251, new_n252,
    new_n253, new_n254, new_n255, new_n256, new_n257, new_n258, new_n259,
    new_n260, new_n261, new_n262, new_n264, new_n265, new_n266, new_n267,
    new_n268, new_n269, new_n271, new_n272, new_n273, new_n274, new_n275,
    new_n276, new_n277, new_n279, new_n280, new_n281, new_n282, new_n283,
    new_n284, new_n285, new_n286, new_n287, new_n288, new_n289, new_n290,
    new_n291, new_n292, new_n294, new_n295, new_n296, new_n297, new_n298,
    new_n299, new_n302, new_n303, new_n304, new_n305, new_n306, new_n307,
    new_n309, new_n310, new_n311, new_n312, new_n313, new_n314, new_n315,
    new_n316, new_n317, new_n319, new_n321, new_n324, new_n326, new_n327,
    new_n328, new_n330, new_n331;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor002aa1n20x5               g001(.a(\b[9] ), .b(\a[10] ), .o1(new_n97));
  nand02aa1d28x5               g002(.a(\b[9] ), .b(\a[10] ), .o1(new_n98));
  nanb02aa1n12x5               g003(.a(new_n97), .b(new_n98), .out0(new_n99));
  orn002aa1n02x5               g004(.a(\a[9] ), .b(\b[8] ), .o(new_n100));
  nor042aa1n06x5               g005(.a(\b[5] ), .b(\a[6] ), .o1(new_n101));
  nand02aa1n06x5               g006(.a(\b[5] ), .b(\a[6] ), .o1(new_n102));
  norp02aa1n24x5               g007(.a(\b[4] ), .b(\a[5] ), .o1(new_n103));
  nand42aa1n02x5               g008(.a(\b[4] ), .b(\a[5] ), .o1(new_n104));
  nona23aa1n09x5               g009(.a(new_n104), .b(new_n102), .c(new_n101), .d(new_n103), .out0(new_n105));
  nor002aa1n02x5               g010(.a(\b[7] ), .b(\a[8] ), .o1(new_n106));
  nand02aa1n04x5               g011(.a(\b[7] ), .b(\a[8] ), .o1(new_n107));
  nor002aa1d32x5               g012(.a(\b[6] ), .b(\a[7] ), .o1(new_n108));
  nand42aa1n04x5               g013(.a(\b[6] ), .b(\a[7] ), .o1(new_n109));
  nona23aa1n03x5               g014(.a(new_n109), .b(new_n107), .c(new_n106), .d(new_n108), .out0(new_n110));
  nor042aa1n04x5               g015(.a(new_n110), .b(new_n105), .o1(new_n111));
  oa0022aa1n06x5               g016(.a(\a[4] ), .b(\b[3] ), .c(\a[3] ), .d(\b[2] ), .o(new_n112));
  nand22aa1n12x5               g017(.a(\b[0] ), .b(\a[1] ), .o1(new_n113));
  nand02aa1n06x5               g018(.a(\b[1] ), .b(\a[2] ), .o1(new_n114));
  nor042aa1n06x5               g019(.a(\b[1] ), .b(\a[2] ), .o1(new_n115));
  nand42aa1n02x5               g020(.a(\b[2] ), .b(\a[3] ), .o1(new_n116));
  oai112aa1n06x5               g021(.a(new_n116), .b(new_n114), .c(new_n115), .d(new_n113), .o1(new_n117));
  aoi022aa1n12x5               g022(.a(new_n117), .b(new_n112), .c(\a[4] ), .d(\b[3] ), .o1(new_n118));
  nanb03aa1n06x5               g023(.a(new_n108), .b(new_n109), .c(new_n107), .out0(new_n119));
  oai122aa1n06x5               g024(.a(new_n102), .b(new_n101), .c(new_n103), .d(\b[7] ), .e(\a[8] ), .o1(new_n120));
  inv040aa1n06x5               g025(.a(new_n108), .o1(new_n121));
  oaoi03aa1n09x5               g026(.a(\a[8] ), .b(\b[7] ), .c(new_n121), .o1(new_n122));
  oabi12aa1n06x5               g027(.a(new_n122), .b(new_n120), .c(new_n119), .out0(new_n123));
  xorc02aa1n12x5               g028(.a(\a[9] ), .b(\b[8] ), .out0(new_n124));
  aoai13aa1n06x5               g029(.a(new_n124), .b(new_n123), .c(new_n118), .d(new_n111), .o1(new_n125));
  xobna2aa1n03x5               g030(.a(new_n99), .b(new_n125), .c(new_n100), .out0(\s[10] ));
  nor002aa1d24x5               g031(.a(\b[10] ), .b(\a[11] ), .o1(new_n127));
  nand02aa1d28x5               g032(.a(\b[10] ), .b(\a[11] ), .o1(new_n128));
  norb02aa1n02x5               g033(.a(new_n128), .b(new_n127), .out0(new_n129));
  nor002aa1n20x5               g034(.a(\b[8] ), .b(\a[9] ), .o1(new_n130));
  nona22aa1n03x5               g035(.a(new_n125), .b(new_n130), .c(new_n97), .out0(new_n131));
  xobna2aa1n03x5               g036(.a(new_n129), .b(new_n131), .c(new_n98), .out0(\s[11] ));
  nor002aa1d32x5               g037(.a(\b[11] ), .b(\a[12] ), .o1(new_n133));
  inv000aa1d42x5               g038(.a(new_n133), .o1(new_n134));
  nand02aa1d24x5               g039(.a(\b[11] ), .b(\a[12] ), .o1(new_n135));
  aoi013aa1n03x5               g040(.a(new_n127), .b(new_n131), .c(new_n128), .d(new_n98), .o1(new_n136));
  xnbna2aa1n03x5               g041(.a(new_n136), .b(new_n134), .c(new_n135), .out0(\s[12] ));
  nona23aa1d18x5               g042(.a(new_n135), .b(new_n128), .c(new_n127), .d(new_n133), .out0(new_n138));
  norb03aa1d15x5               g043(.a(new_n124), .b(new_n138), .c(new_n99), .out0(new_n139));
  aoai13aa1n06x5               g044(.a(new_n139), .b(new_n123), .c(new_n118), .d(new_n111), .o1(new_n140));
  nanb03aa1n12x5               g045(.a(new_n133), .b(new_n135), .c(new_n128), .out0(new_n141));
  oai122aa1n12x5               g046(.a(new_n98), .b(new_n97), .c(new_n130), .d(\b[10] ), .e(\a[11] ), .o1(new_n142));
  aoi012aa1d18x5               g047(.a(new_n133), .b(new_n127), .c(new_n135), .o1(new_n143));
  oai012aa1d24x5               g048(.a(new_n143), .b(new_n142), .c(new_n141), .o1(new_n144));
  inv000aa1d42x5               g049(.a(new_n144), .o1(new_n145));
  nor002aa1d32x5               g050(.a(\b[12] ), .b(\a[13] ), .o1(new_n146));
  nand22aa1n06x5               g051(.a(\b[12] ), .b(\a[13] ), .o1(new_n147));
  nanb02aa1n02x5               g052(.a(new_n146), .b(new_n147), .out0(new_n148));
  xobna2aa1n03x5               g053(.a(new_n148), .b(new_n140), .c(new_n145), .out0(\s[13] ));
  nanp02aa1n02x5               g054(.a(new_n140), .b(new_n145), .o1(new_n150));
  aoi012aa1n02x5               g055(.a(new_n146), .b(new_n150), .c(new_n147), .o1(new_n151));
  xnrb03aa1n02x5               g056(.a(new_n151), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  nor002aa1d24x5               g057(.a(\b[13] ), .b(\a[14] ), .o1(new_n153));
  nand22aa1n09x5               g058(.a(\b[13] ), .b(\a[14] ), .o1(new_n154));
  nona23aa1d18x5               g059(.a(new_n154), .b(new_n147), .c(new_n146), .d(new_n153), .out0(new_n155));
  aoi012aa1d18x5               g060(.a(new_n153), .b(new_n146), .c(new_n154), .o1(new_n156));
  aoai13aa1n06x5               g061(.a(new_n156), .b(new_n155), .c(new_n140), .d(new_n145), .o1(new_n157));
  xorb03aa1n02x5               g062(.a(new_n157), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  inv000aa1d42x5               g063(.a(\a[16] ), .o1(new_n159));
  norp02aa1n02x5               g064(.a(\b[14] ), .b(\a[15] ), .o1(new_n160));
  xnrc02aa1n12x5               g065(.a(\b[14] ), .b(\a[15] ), .out0(new_n161));
  inv000aa1d42x5               g066(.a(new_n161), .o1(new_n162));
  aoi012aa1n03x5               g067(.a(new_n160), .b(new_n157), .c(new_n162), .o1(new_n163));
  xorb03aa1n02x5               g068(.a(new_n163), .b(\b[15] ), .c(new_n159), .out0(\s[16] ));
  tech160nm_fiaoi012aa1n05x5   g069(.a(new_n123), .b(new_n118), .c(new_n111), .o1(new_n165));
  xnrc02aa1n02x5               g070(.a(\b[15] ), .b(\a[16] ), .out0(new_n166));
  nona32aa1n09x5               g071(.a(new_n139), .b(new_n166), .c(new_n161), .d(new_n155), .out0(new_n167));
  inv040aa1n06x5               g072(.a(new_n155), .o1(new_n168));
  inv000aa1n02x5               g073(.a(new_n156), .o1(new_n169));
  nor042aa1n03x5               g074(.a(new_n166), .b(new_n161), .o1(new_n170));
  aoai13aa1n04x5               g075(.a(new_n170), .b(new_n169), .c(new_n144), .d(new_n168), .o1(new_n171));
  aoi112aa1n02x5               g076(.a(\b[14] ), .b(\a[15] ), .c(\a[16] ), .d(\b[15] ), .o1(new_n172));
  aoib12aa1n06x5               g077(.a(new_n172), .b(new_n159), .c(\b[15] ), .out0(new_n173));
  oai112aa1n06x5               g078(.a(new_n171), .b(new_n173), .c(new_n165), .d(new_n167), .o1(new_n174));
  xorb03aa1n02x5               g079(.a(new_n174), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  nand02aa1d06x5               g080(.a(new_n118), .b(new_n111), .o1(new_n176));
  oab012aa1n06x5               g081(.a(new_n122), .b(new_n120), .c(new_n119), .out0(new_n177));
  aoi012aa1d24x5               g082(.a(new_n167), .b(new_n176), .c(new_n177), .o1(new_n178));
  inv040aa1n02x5               g083(.a(new_n170), .o1(new_n179));
  nano22aa1n03x7               g084(.a(new_n133), .b(new_n128), .c(new_n135), .out0(new_n180));
  oai012aa1n02x7               g085(.a(new_n98), .b(\b[10] ), .c(\a[11] ), .o1(new_n181));
  oab012aa1n03x5               g086(.a(new_n181), .b(new_n97), .c(new_n130), .out0(new_n182));
  inv020aa1n03x5               g087(.a(new_n143), .o1(new_n183));
  aoai13aa1n04x5               g088(.a(new_n168), .b(new_n183), .c(new_n182), .d(new_n180), .o1(new_n184));
  aoai13aa1n12x5               g089(.a(new_n173), .b(new_n179), .c(new_n184), .d(new_n156), .o1(new_n185));
  nor042aa1n04x5               g090(.a(\b[16] ), .b(\a[17] ), .o1(new_n186));
  nanp02aa1n04x5               g091(.a(\b[16] ), .b(\a[17] ), .o1(new_n187));
  oaoi13aa1n04x5               g092(.a(new_n186), .b(new_n187), .c(new_n185), .d(new_n178), .o1(new_n188));
  xnrb03aa1n03x5               g093(.a(new_n188), .b(\b[17] ), .c(\a[18] ), .out0(\s[18] ));
  nor042aa1n02x5               g094(.a(\b[17] ), .b(\a[18] ), .o1(new_n190));
  nand02aa1n06x5               g095(.a(\b[17] ), .b(\a[18] ), .o1(new_n191));
  nano23aa1n06x5               g096(.a(new_n186), .b(new_n190), .c(new_n191), .d(new_n187), .out0(new_n192));
  tech160nm_fioai012aa1n03p5x5 g097(.a(new_n192), .b(new_n185), .c(new_n178), .o1(new_n193));
  oa0012aa1n02x5               g098(.a(new_n191), .b(new_n190), .c(new_n186), .o(new_n194));
  inv000aa1n02x5               g099(.a(new_n194), .o1(new_n195));
  nor042aa1n06x5               g100(.a(\b[18] ), .b(\a[19] ), .o1(new_n196));
  nand02aa1n08x5               g101(.a(\b[18] ), .b(\a[19] ), .o1(new_n197));
  norb02aa1n03x5               g102(.a(new_n197), .b(new_n196), .out0(new_n198));
  xnbna2aa1n03x5               g103(.a(new_n198), .b(new_n193), .c(new_n195), .out0(\s[19] ));
  xnrc02aa1n02x5               g104(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nanp02aa1n02x5               g105(.a(new_n193), .b(new_n195), .o1(new_n201));
  nor042aa1n06x5               g106(.a(\b[19] ), .b(\a[20] ), .o1(new_n202));
  nand02aa1d28x5               g107(.a(\b[19] ), .b(\a[20] ), .o1(new_n203));
  nanb02aa1n02x5               g108(.a(new_n202), .b(new_n203), .out0(new_n204));
  aoai13aa1n03x5               g109(.a(new_n204), .b(new_n196), .c(new_n201), .d(new_n197), .o1(new_n205));
  aoai13aa1n03x5               g110(.a(new_n198), .b(new_n194), .c(new_n174), .d(new_n192), .o1(new_n206));
  nona22aa1n02x5               g111(.a(new_n206), .b(new_n204), .c(new_n196), .out0(new_n207));
  nanp02aa1n03x5               g112(.a(new_n205), .b(new_n207), .o1(\s[20] ));
  nano22aa1n03x7               g113(.a(new_n204), .b(new_n192), .c(new_n198), .out0(new_n209));
  oai012aa1n06x5               g114(.a(new_n209), .b(new_n185), .c(new_n178), .o1(new_n210));
  nanb03aa1n12x5               g115(.a(new_n202), .b(new_n203), .c(new_n197), .out0(new_n211));
  oaih22aa1n06x5               g116(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n212));
  inv000aa1d42x5               g117(.a(\b[18] ), .o1(new_n213));
  nanb02aa1n03x5               g118(.a(\a[19] ), .b(new_n213), .out0(new_n214));
  nand23aa1n06x5               g119(.a(new_n212), .b(new_n214), .c(new_n191), .o1(new_n215));
  tech160nm_fiaoi012aa1n05x5   g120(.a(new_n202), .b(new_n196), .c(new_n203), .o1(new_n216));
  oai012aa1d24x5               g121(.a(new_n216), .b(new_n215), .c(new_n211), .o1(new_n217));
  inv000aa1d42x5               g122(.a(new_n217), .o1(new_n218));
  xnrc02aa1n12x5               g123(.a(\b[20] ), .b(\a[21] ), .out0(new_n219));
  inv000aa1d42x5               g124(.a(new_n219), .o1(new_n220));
  xnbna2aa1n03x5               g125(.a(new_n220), .b(new_n210), .c(new_n218), .out0(\s[21] ));
  nand42aa1n03x5               g126(.a(new_n210), .b(new_n218), .o1(new_n222));
  nor042aa1n03x5               g127(.a(\b[20] ), .b(\a[21] ), .o1(new_n223));
  tech160nm_fixnrc02aa1n05x5   g128(.a(\b[21] ), .b(\a[22] ), .out0(new_n224));
  aoai13aa1n03x5               g129(.a(new_n224), .b(new_n223), .c(new_n222), .d(new_n220), .o1(new_n225));
  aoai13aa1n03x5               g130(.a(new_n220), .b(new_n217), .c(new_n174), .d(new_n209), .o1(new_n226));
  nona22aa1n03x5               g131(.a(new_n226), .b(new_n224), .c(new_n223), .out0(new_n227));
  nanp02aa1n03x5               g132(.a(new_n225), .b(new_n227), .o1(\s[22] ));
  nor042aa1n09x5               g133(.a(new_n224), .b(new_n219), .o1(new_n229));
  inv000aa1d42x5               g134(.a(new_n229), .o1(new_n230));
  nano23aa1n03x7               g135(.a(new_n230), .b(new_n204), .c(new_n198), .d(new_n192), .out0(new_n231));
  tech160nm_fioai012aa1n03p5x5 g136(.a(new_n231), .b(new_n185), .c(new_n178), .o1(new_n232));
  inv000aa1d42x5               g137(.a(\a[22] ), .o1(new_n233));
  inv000aa1d42x5               g138(.a(\b[21] ), .o1(new_n234));
  oaoi03aa1n09x5               g139(.a(new_n233), .b(new_n234), .c(new_n223), .o1(new_n235));
  inv000aa1d42x5               g140(.a(new_n235), .o1(new_n236));
  tech160nm_fiaoi012aa1n05x5   g141(.a(new_n236), .b(new_n217), .c(new_n229), .o1(new_n237));
  xorc02aa1n12x5               g142(.a(\a[23] ), .b(\b[22] ), .out0(new_n238));
  xnbna2aa1n03x5               g143(.a(new_n238), .b(new_n232), .c(new_n237), .out0(\s[23] ));
  nand42aa1n03x5               g144(.a(new_n232), .b(new_n237), .o1(new_n240));
  norp02aa1n02x5               g145(.a(\b[22] ), .b(\a[23] ), .o1(new_n241));
  tech160nm_fixnrc02aa1n04x5   g146(.a(\b[23] ), .b(\a[24] ), .out0(new_n242));
  aoai13aa1n03x5               g147(.a(new_n242), .b(new_n241), .c(new_n240), .d(new_n238), .o1(new_n243));
  inv000aa1n02x5               g148(.a(new_n237), .o1(new_n244));
  aoai13aa1n03x5               g149(.a(new_n238), .b(new_n244), .c(new_n174), .d(new_n231), .o1(new_n245));
  nona22aa1n03x5               g150(.a(new_n245), .b(new_n242), .c(new_n241), .out0(new_n246));
  nanp02aa1n03x5               g151(.a(new_n243), .b(new_n246), .o1(\s[24] ));
  inv040aa1n03x5               g152(.a(new_n209), .o1(new_n248));
  norb02aa1n03x5               g153(.a(new_n238), .b(new_n242), .out0(new_n249));
  nano22aa1n03x7               g154(.a(new_n248), .b(new_n229), .c(new_n249), .out0(new_n250));
  oaih12aa1n02x5               g155(.a(new_n250), .b(new_n185), .c(new_n178), .o1(new_n251));
  nano22aa1n02x5               g156(.a(new_n202), .b(new_n197), .c(new_n203), .out0(new_n252));
  oai012aa1n02x5               g157(.a(new_n191), .b(\b[18] ), .c(\a[19] ), .o1(new_n253));
  oab012aa1n06x5               g158(.a(new_n253), .b(new_n186), .c(new_n190), .out0(new_n254));
  inv000aa1n02x5               g159(.a(new_n216), .o1(new_n255));
  aoai13aa1n06x5               g160(.a(new_n229), .b(new_n255), .c(new_n254), .d(new_n252), .o1(new_n256));
  inv000aa1n02x5               g161(.a(new_n249), .o1(new_n257));
  orn002aa1n02x5               g162(.a(\a[23] ), .b(\b[22] ), .o(new_n258));
  oao003aa1n02x5               g163(.a(\a[24] ), .b(\b[23] ), .c(new_n258), .carry(new_n259));
  aoai13aa1n06x5               g164(.a(new_n259), .b(new_n257), .c(new_n256), .d(new_n235), .o1(new_n260));
  inv040aa1n03x5               g165(.a(new_n260), .o1(new_n261));
  xorc02aa1n12x5               g166(.a(\a[25] ), .b(\b[24] ), .out0(new_n262));
  xnbna2aa1n03x5               g167(.a(new_n262), .b(new_n251), .c(new_n261), .out0(\s[25] ));
  nanp02aa1n03x5               g168(.a(new_n251), .b(new_n261), .o1(new_n264));
  norp02aa1n02x5               g169(.a(\b[24] ), .b(\a[25] ), .o1(new_n265));
  tech160nm_fixnrc02aa1n04x5   g170(.a(\b[25] ), .b(\a[26] ), .out0(new_n266));
  aoai13aa1n03x5               g171(.a(new_n266), .b(new_n265), .c(new_n264), .d(new_n262), .o1(new_n267));
  aoai13aa1n03x5               g172(.a(new_n262), .b(new_n260), .c(new_n174), .d(new_n250), .o1(new_n268));
  nona22aa1n03x5               g173(.a(new_n268), .b(new_n266), .c(new_n265), .out0(new_n269));
  nanp02aa1n03x5               g174(.a(new_n267), .b(new_n269), .o1(\s[26] ));
  norb02aa1n06x5               g175(.a(new_n262), .b(new_n266), .out0(new_n271));
  nano32aa1n06x5               g176(.a(new_n248), .b(new_n271), .c(new_n229), .d(new_n249), .out0(new_n272));
  oaih12aa1n02x5               g177(.a(new_n272), .b(new_n185), .c(new_n178), .o1(new_n273));
  orn002aa1n02x5               g178(.a(\a[25] ), .b(\b[24] ), .o(new_n274));
  oao003aa1n02x5               g179(.a(\a[26] ), .b(\b[25] ), .c(new_n274), .carry(new_n275));
  aobi12aa1n06x5               g180(.a(new_n275), .b(new_n260), .c(new_n271), .out0(new_n276));
  xorc02aa1n12x5               g181(.a(\a[27] ), .b(\b[26] ), .out0(new_n277));
  xnbna2aa1n03x5               g182(.a(new_n277), .b(new_n276), .c(new_n273), .out0(\s[27] ));
  norp02aa1n02x5               g183(.a(new_n185), .b(new_n178), .o1(new_n279));
  inv000aa1n02x5               g184(.a(new_n272), .o1(new_n280));
  oaih12aa1n02x5               g185(.a(new_n276), .b(new_n279), .c(new_n280), .o1(new_n281));
  norp02aa1n02x5               g186(.a(\b[26] ), .b(\a[27] ), .o1(new_n282));
  norp02aa1n02x5               g187(.a(\b[27] ), .b(\a[28] ), .o1(new_n283));
  nand42aa1n03x5               g188(.a(\b[27] ), .b(\a[28] ), .o1(new_n284));
  norb02aa1n06x4               g189(.a(new_n284), .b(new_n283), .out0(new_n285));
  inv040aa1n03x5               g190(.a(new_n285), .o1(new_n286));
  aoai13aa1n03x5               g191(.a(new_n286), .b(new_n282), .c(new_n281), .d(new_n277), .o1(new_n287));
  aoai13aa1n04x5               g192(.a(new_n249), .b(new_n236), .c(new_n217), .d(new_n229), .o1(new_n288));
  inv000aa1d42x5               g193(.a(new_n271), .o1(new_n289));
  aoai13aa1n06x5               g194(.a(new_n275), .b(new_n289), .c(new_n288), .d(new_n259), .o1(new_n290));
  aoai13aa1n02x7               g195(.a(new_n277), .b(new_n290), .c(new_n174), .d(new_n272), .o1(new_n291));
  nona22aa1n02x4               g196(.a(new_n291), .b(new_n286), .c(new_n282), .out0(new_n292));
  nanp02aa1n03x5               g197(.a(new_n287), .b(new_n292), .o1(\s[28] ));
  norb02aa1n02x5               g198(.a(new_n277), .b(new_n286), .out0(new_n294));
  aoai13aa1n03x5               g199(.a(new_n294), .b(new_n290), .c(new_n174), .d(new_n272), .o1(new_n295));
  oai012aa1n02x5               g200(.a(new_n284), .b(new_n283), .c(new_n282), .o1(new_n296));
  nanp02aa1n03x5               g201(.a(new_n295), .b(new_n296), .o1(new_n297));
  xorc02aa1n02x5               g202(.a(\a[29] ), .b(\b[28] ), .out0(new_n298));
  norb02aa1n02x5               g203(.a(new_n296), .b(new_n298), .out0(new_n299));
  aoi022aa1n02x7               g204(.a(new_n297), .b(new_n298), .c(new_n295), .d(new_n299), .o1(\s[29] ));
  xorb03aa1n02x5               g205(.a(new_n113), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  and003aa1n02x5               g206(.a(new_n277), .b(new_n298), .c(new_n285), .o(new_n302));
  aoai13aa1n03x5               g207(.a(new_n302), .b(new_n290), .c(new_n174), .d(new_n272), .o1(new_n303));
  oao003aa1n02x5               g208(.a(\a[29] ), .b(\b[28] ), .c(new_n296), .carry(new_n304));
  nanp02aa1n03x5               g209(.a(new_n303), .b(new_n304), .o1(new_n305));
  xorc02aa1n02x5               g210(.a(\a[30] ), .b(\b[29] ), .out0(new_n306));
  norb02aa1n02x5               g211(.a(new_n304), .b(new_n306), .out0(new_n307));
  aoi022aa1n02x7               g212(.a(new_n305), .b(new_n306), .c(new_n303), .d(new_n307), .o1(\s[30] ));
  nanb02aa1n02x5               g213(.a(\b[30] ), .b(\a[31] ), .out0(new_n309));
  nanb02aa1n02x5               g214(.a(\a[31] ), .b(\b[30] ), .out0(new_n310));
  and003aa1n06x5               g215(.a(new_n294), .b(new_n306), .c(new_n298), .o(new_n311));
  aoai13aa1n02x7               g216(.a(new_n311), .b(new_n290), .c(new_n174), .d(new_n272), .o1(new_n312));
  oao003aa1n02x5               g217(.a(\a[30] ), .b(\b[29] ), .c(new_n304), .carry(new_n313));
  aoi022aa1n02x5               g218(.a(new_n312), .b(new_n313), .c(new_n310), .d(new_n309), .o1(new_n314));
  inv000aa1n02x5               g219(.a(new_n311), .o1(new_n315));
  aoi012aa1n03x5               g220(.a(new_n315), .b(new_n276), .c(new_n273), .o1(new_n316));
  nano32aa1n03x5               g221(.a(new_n316), .b(new_n313), .c(new_n309), .d(new_n310), .out0(new_n317));
  nor002aa1n02x5               g222(.a(new_n314), .b(new_n317), .o1(\s[31] ));
  oai012aa1n02x5               g223(.a(new_n114), .b(new_n115), .c(new_n113), .o1(new_n319));
  xnrb03aa1n02x5               g224(.a(new_n319), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g225(.a(\a[3] ), .b(\b[2] ), .c(new_n319), .o1(new_n321));
  xorb03aa1n02x5               g226(.a(new_n321), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g227(.a(new_n118), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  oai012aa1n03x5               g228(.a(new_n104), .b(new_n118), .c(new_n103), .o1(new_n324));
  xnrb03aa1n02x5               g229(.a(new_n324), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  nano22aa1n03x7               g230(.a(new_n101), .b(new_n324), .c(new_n102), .out0(new_n326));
  aboi22aa1n03x5               g231(.a(new_n326), .b(new_n102), .c(new_n121), .d(new_n109), .out0(new_n327));
  nano32aa1n03x7               g232(.a(new_n326), .b(new_n109), .c(new_n121), .d(new_n102), .out0(new_n328));
  norp02aa1n02x5               g233(.a(new_n327), .b(new_n328), .o1(\s[7] ));
  orn002aa1n02x5               g234(.a(\a[8] ), .b(\b[7] ), .o(new_n330));
  nor042aa1n03x5               g235(.a(new_n328), .b(new_n108), .o1(new_n331));
  xnbna2aa1n03x5               g236(.a(new_n331), .b(new_n330), .c(new_n107), .out0(\s[8] ));
  xnbna2aa1n03x5               g237(.a(new_n124), .b(new_n176), .c(new_n177), .out0(\s[9] ));
endmodule


