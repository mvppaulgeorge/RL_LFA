// Benchmark "adder" written by ABC on Thu Jul 18 06:36:02 2024

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
    new_n125, new_n126, new_n128, new_n129, new_n130, new_n131, new_n132,
    new_n133, new_n134, new_n135, new_n137, new_n139, new_n140, new_n141,
    new_n142, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n160, new_n161, new_n162, new_n164,
    new_n165, new_n166, new_n167, new_n168, new_n169, new_n170, new_n171,
    new_n172, new_n173, new_n174, new_n176, new_n177, new_n178, new_n179,
    new_n180, new_n181, new_n182, new_n183, new_n184, new_n185, new_n186,
    new_n188, new_n189, new_n190, new_n191, new_n192, new_n193, new_n194,
    new_n195, new_n196, new_n197, new_n199, new_n200, new_n201, new_n202,
    new_n203, new_n205, new_n206, new_n207, new_n208, new_n209, new_n210,
    new_n211, new_n214, new_n215, new_n216, new_n217, new_n218, new_n219,
    new_n220, new_n221, new_n222, new_n223, new_n224, new_n225, new_n227,
    new_n228, new_n229, new_n230, new_n231, new_n232, new_n233, new_n234,
    new_n236, new_n237, new_n238, new_n239, new_n240, new_n241, new_n242,
    new_n243, new_n244, new_n245, new_n247, new_n248, new_n249, new_n250,
    new_n251, new_n252, new_n253, new_n254, new_n255, new_n256, new_n258,
    new_n259, new_n260, new_n261, new_n262, new_n263, new_n264, new_n265,
    new_n266, new_n267, new_n269, new_n270, new_n271, new_n272, new_n273,
    new_n274, new_n275, new_n276, new_n277, new_n278, new_n280, new_n281,
    new_n282, new_n283, new_n284, new_n285, new_n286, new_n287, new_n289,
    new_n290, new_n291, new_n292, new_n293, new_n294, new_n295, new_n297,
    new_n298, new_n299, new_n300, new_n301, new_n302, new_n303, new_n304,
    new_n305, new_n306, new_n308, new_n309, new_n310, new_n311, new_n312,
    new_n313, new_n314, new_n317, new_n318, new_n319, new_n320, new_n321,
    new_n322, new_n323, new_n325, new_n326, new_n327, new_n328, new_n329,
    new_n330, new_n331, new_n332, new_n335, new_n338, new_n340, new_n341,
    new_n343;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor022aa1n08x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  inv000aa1d42x5               g002(.a(\b[2] ), .o1(new_n98));
  nanb02aa1d24x5               g003(.a(\a[3] ), .b(new_n98), .out0(new_n99));
  oaoi03aa1n09x5               g004(.a(\a[4] ), .b(\b[3] ), .c(new_n99), .o1(new_n100));
  inv030aa1n03x5               g005(.a(new_n100), .o1(new_n101));
  xorc02aa1n02x5               g006(.a(\a[4] ), .b(\b[3] ), .out0(new_n102));
  nor002aa1n16x5               g007(.a(\b[1] ), .b(\a[2] ), .o1(new_n103));
  inv000aa1n02x5               g008(.a(new_n103), .o1(new_n104));
  nand02aa1d06x5               g009(.a(\b[0] ), .b(\a[1] ), .o1(new_n105));
  nand02aa1n06x5               g010(.a(\b[1] ), .b(\a[2] ), .o1(new_n106));
  aob012aa1n02x5               g011(.a(new_n104), .b(new_n105), .c(new_n106), .out0(new_n107));
  norp02aa1n02x5               g012(.a(\b[2] ), .b(\a[3] ), .o1(new_n108));
  nand22aa1n04x5               g013(.a(\b[2] ), .b(\a[3] ), .o1(new_n109));
  norb02aa1n02x5               g014(.a(new_n109), .b(new_n108), .out0(new_n110));
  nand23aa1n03x5               g015(.a(new_n107), .b(new_n102), .c(new_n110), .o1(new_n111));
  xorc02aa1n02x5               g016(.a(\a[6] ), .b(\b[5] ), .out0(new_n112));
  xorc02aa1n02x5               g017(.a(\a[5] ), .b(\b[4] ), .out0(new_n113));
  nand02aa1n12x5               g018(.a(\b[6] ), .b(\a[7] ), .o1(new_n114));
  nor002aa1d32x5               g019(.a(\b[7] ), .b(\a[8] ), .o1(new_n115));
  nand22aa1n12x5               g020(.a(\b[7] ), .b(\a[8] ), .o1(new_n116));
  nor022aa1n16x5               g021(.a(\b[6] ), .b(\a[7] ), .o1(new_n117));
  nano23aa1n09x5               g022(.a(new_n117), .b(new_n115), .c(new_n114), .d(new_n116), .out0(new_n118));
  nand03aa1n02x5               g023(.a(new_n118), .b(new_n112), .c(new_n113), .o1(new_n119));
  orn002aa1n24x5               g024(.a(\a[5] ), .b(\b[4] ), .o(new_n120));
  oaoi03aa1n12x5               g025(.a(\a[6] ), .b(\b[5] ), .c(new_n120), .o1(new_n121));
  tech160nm_fioai012aa1n04x5   g026(.a(new_n116), .b(new_n117), .c(new_n115), .o1(new_n122));
  aobi12aa1n06x5               g027(.a(new_n122), .b(new_n118), .c(new_n121), .out0(new_n123));
  aoai13aa1n06x5               g028(.a(new_n123), .b(new_n119), .c(new_n111), .d(new_n101), .o1(new_n124));
  nand42aa1n10x5               g029(.a(\b[8] ), .b(\a[9] ), .o1(new_n125));
  aoi012aa1n02x5               g030(.a(new_n97), .b(new_n124), .c(new_n125), .o1(new_n126));
  xnrb03aa1n03x5               g031(.a(new_n126), .b(\b[9] ), .c(\a[10] ), .out0(\s[10] ));
  nor042aa1n06x5               g032(.a(\b[10] ), .b(\a[11] ), .o1(new_n128));
  nand02aa1n08x5               g033(.a(\b[10] ), .b(\a[11] ), .o1(new_n129));
  norb02aa1n02x5               g034(.a(new_n129), .b(new_n128), .out0(new_n130));
  nanp02aa1n12x5               g035(.a(\b[9] ), .b(\a[10] ), .o1(new_n131));
  oai022aa1d18x5               g036(.a(\a[10] ), .b(\b[9] ), .c(\b[8] ), .d(\a[9] ), .o1(new_n132));
  nor022aa1n08x5               g037(.a(\b[9] ), .b(\a[10] ), .o1(new_n133));
  nano23aa1n03x7               g038(.a(new_n97), .b(new_n133), .c(new_n131), .d(new_n125), .out0(new_n134));
  aoi022aa1n03x5               g039(.a(new_n124), .b(new_n134), .c(new_n131), .d(new_n132), .o1(new_n135));
  xnrc02aa1n02x5               g040(.a(new_n135), .b(new_n130), .out0(\s[11] ));
  oaoi03aa1n03x5               g041(.a(\a[11] ), .b(\b[10] ), .c(new_n135), .o1(new_n137));
  xorb03aa1n02x5               g042(.a(new_n137), .b(\b[11] ), .c(\a[12] ), .out0(\s[12] ));
  nanp02aa1n02x5               g043(.a(\b[3] ), .b(\a[4] ), .o1(new_n139));
  orn002aa1n03x5               g044(.a(\a[4] ), .b(\b[3] ), .o(new_n140));
  nand02aa1n04x5               g045(.a(new_n140), .b(new_n139), .o1(new_n141));
  aoi012aa1n12x5               g046(.a(new_n103), .b(new_n105), .c(new_n106), .o1(new_n142));
  nanp02aa1n04x5               g047(.a(new_n99), .b(new_n109), .o1(new_n143));
  norp03aa1n02x5               g048(.a(new_n142), .b(new_n141), .c(new_n143), .o1(new_n144));
  tech160nm_fixnrc02aa1n04x5   g049(.a(\b[5] ), .b(\a[6] ), .out0(new_n145));
  tech160nm_fixnrc02aa1n05x5   g050(.a(\b[4] ), .b(\a[5] ), .out0(new_n146));
  nona23aa1n09x5               g051(.a(new_n116), .b(new_n114), .c(new_n117), .d(new_n115), .out0(new_n147));
  nor043aa1n03x5               g052(.a(new_n147), .b(new_n146), .c(new_n145), .o1(new_n148));
  oai012aa1n03x5               g053(.a(new_n148), .b(new_n144), .c(new_n100), .o1(new_n149));
  nanb02aa1n03x5               g054(.a(new_n128), .b(new_n129), .out0(new_n150));
  nor002aa1n03x5               g055(.a(\b[11] ), .b(\a[12] ), .o1(new_n151));
  nand02aa1d08x5               g056(.a(\b[11] ), .b(\a[12] ), .o1(new_n152));
  nanb02aa1n03x5               g057(.a(new_n151), .b(new_n152), .out0(new_n153));
  nona22aa1n03x5               g058(.a(new_n134), .b(new_n153), .c(new_n150), .out0(new_n154));
  nanp03aa1n06x5               g059(.a(new_n132), .b(new_n131), .c(new_n129), .o1(new_n155));
  nona22aa1n09x5               g060(.a(new_n155), .b(new_n151), .c(new_n128), .out0(new_n156));
  nand42aa1n02x5               g061(.a(new_n156), .b(new_n152), .o1(new_n157));
  aoai13aa1n03x5               g062(.a(new_n157), .b(new_n154), .c(new_n149), .d(new_n123), .o1(new_n158));
  xorb03aa1n02x5               g063(.a(new_n158), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  nor002aa1d32x5               g064(.a(\b[12] ), .b(\a[13] ), .o1(new_n160));
  nand02aa1d28x5               g065(.a(\b[12] ), .b(\a[13] ), .o1(new_n161));
  aoi012aa1n03x5               g066(.a(new_n160), .b(new_n158), .c(new_n161), .o1(new_n162));
  xnrb03aa1n03x5               g067(.a(new_n162), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  oai013aa1n03x5               g068(.a(new_n101), .b(new_n142), .c(new_n141), .d(new_n143), .o1(new_n164));
  oaib12aa1n06x5               g069(.a(new_n122), .b(new_n147), .c(new_n121), .out0(new_n165));
  nona23aa1n02x4               g070(.a(new_n131), .b(new_n125), .c(new_n97), .d(new_n133), .out0(new_n166));
  norp03aa1n02x5               g071(.a(new_n166), .b(new_n153), .c(new_n150), .o1(new_n167));
  aoai13aa1n03x5               g072(.a(new_n167), .b(new_n165), .c(new_n164), .d(new_n148), .o1(new_n168));
  nor002aa1d32x5               g073(.a(\b[13] ), .b(\a[14] ), .o1(new_n169));
  nand02aa1d12x5               g074(.a(\b[13] ), .b(\a[14] ), .o1(new_n170));
  nona23aa1n09x5               g075(.a(new_n170), .b(new_n161), .c(new_n160), .d(new_n169), .out0(new_n171));
  oa0012aa1n02x5               g076(.a(new_n170), .b(new_n169), .c(new_n160), .o(new_n172));
  inv000aa1n02x5               g077(.a(new_n172), .o1(new_n173));
  aoai13aa1n02x7               g078(.a(new_n173), .b(new_n171), .c(new_n168), .d(new_n157), .o1(new_n174));
  xorb03aa1n02x5               g079(.a(new_n174), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  nor042aa1n06x5               g080(.a(\b[14] ), .b(\a[15] ), .o1(new_n176));
  inv030aa1n02x5               g081(.a(new_n176), .o1(new_n177));
  nano23aa1n06x5               g082(.a(new_n160), .b(new_n169), .c(new_n170), .d(new_n161), .out0(new_n178));
  nand22aa1n12x5               g083(.a(\b[14] ), .b(\a[15] ), .o1(new_n179));
  norb02aa1n06x5               g084(.a(new_n179), .b(new_n176), .out0(new_n180));
  aoai13aa1n03x5               g085(.a(new_n180), .b(new_n172), .c(new_n158), .d(new_n178), .o1(new_n181));
  nor042aa1d18x5               g086(.a(\b[15] ), .b(\a[16] ), .o1(new_n182));
  nand22aa1n12x5               g087(.a(\b[15] ), .b(\a[16] ), .o1(new_n183));
  norb02aa1n15x5               g088(.a(new_n183), .b(new_n182), .out0(new_n184));
  aobi12aa1n02x7               g089(.a(new_n184), .b(new_n181), .c(new_n177), .out0(new_n185));
  aoi112aa1n02x5               g090(.a(new_n176), .b(new_n184), .c(new_n174), .d(new_n179), .o1(new_n186));
  nor002aa1n02x5               g091(.a(new_n185), .b(new_n186), .o1(\s[16] ));
  nand03aa1n02x5               g092(.a(new_n178), .b(new_n180), .c(new_n184), .o1(new_n188));
  nor042aa1n03x5               g093(.a(new_n188), .b(new_n154), .o1(new_n189));
  aoai13aa1n06x5               g094(.a(new_n189), .b(new_n165), .c(new_n164), .d(new_n148), .o1(new_n190));
  nano22aa1n03x7               g095(.a(new_n171), .b(new_n180), .c(new_n184), .out0(new_n191));
  inv000aa1n02x5               g096(.a(new_n182), .o1(new_n192));
  inv000aa1n02x5               g097(.a(new_n183), .o1(new_n193));
  oai112aa1n04x5               g098(.a(new_n170), .b(new_n179), .c(new_n169), .d(new_n160), .o1(new_n194));
  aoai13aa1n06x5               g099(.a(new_n192), .b(new_n193), .c(new_n194), .d(new_n177), .o1(new_n195));
  aoi013aa1n09x5               g100(.a(new_n195), .b(new_n191), .c(new_n156), .d(new_n152), .o1(new_n196));
  xorc02aa1n12x5               g101(.a(\a[17] ), .b(\b[16] ), .out0(new_n197));
  xnbna2aa1n03x5               g102(.a(new_n197), .b(new_n190), .c(new_n196), .out0(\s[17] ));
  nor002aa1d32x5               g103(.a(\b[16] ), .b(\a[17] ), .o1(new_n199));
  inv020aa1n04x5               g104(.a(new_n199), .o1(new_n200));
  oabi12aa1n02x5               g105(.a(new_n195), .b(new_n157), .c(new_n188), .out0(new_n201));
  aoai13aa1n03x5               g106(.a(new_n197), .b(new_n201), .c(new_n124), .d(new_n189), .o1(new_n202));
  xnrc02aa1n12x5               g107(.a(\b[17] ), .b(\a[18] ), .out0(new_n203));
  xobna2aa1n03x5               g108(.a(new_n203), .b(new_n202), .c(new_n200), .out0(\s[18] ));
  inv030aa1d32x5               g109(.a(\a[17] ), .o1(new_n205));
  inv020aa1n04x5               g110(.a(\a[18] ), .o1(new_n206));
  xroi22aa1d06x4               g111(.a(new_n205), .b(\b[16] ), .c(new_n206), .d(\b[17] ), .out0(new_n207));
  inv000aa1n02x5               g112(.a(new_n207), .o1(new_n208));
  oaoi03aa1n02x5               g113(.a(\a[18] ), .b(\b[17] ), .c(new_n200), .o1(new_n209));
  inv000aa1n02x5               g114(.a(new_n209), .o1(new_n210));
  aoai13aa1n06x5               g115(.a(new_n210), .b(new_n208), .c(new_n190), .d(new_n196), .o1(new_n211));
  xorb03aa1n02x5               g116(.a(new_n211), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g117(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor002aa1d24x5               g118(.a(\b[18] ), .b(\a[19] ), .o1(new_n214));
  nand22aa1n09x5               g119(.a(\b[18] ), .b(\a[19] ), .o1(new_n215));
  xnrc02aa1n12x5               g120(.a(\b[19] ), .b(\a[20] ), .out0(new_n216));
  inv030aa1n02x5               g121(.a(new_n216), .o1(new_n217));
  aoi112aa1n03x4               g122(.a(new_n214), .b(new_n217), .c(new_n211), .d(new_n215), .o1(new_n218));
  inv000aa1d42x5               g123(.a(new_n214), .o1(new_n219));
  nona32aa1n02x4               g124(.a(new_n191), .b(new_n166), .c(new_n153), .d(new_n150), .out0(new_n220));
  aoai13aa1n06x5               g125(.a(new_n196), .b(new_n220), .c(new_n149), .d(new_n123), .o1(new_n221));
  nanb02aa1n18x5               g126(.a(new_n214), .b(new_n215), .out0(new_n222));
  inv000aa1d42x5               g127(.a(new_n222), .o1(new_n223));
  aoai13aa1n03x5               g128(.a(new_n223), .b(new_n209), .c(new_n221), .d(new_n207), .o1(new_n224));
  aoi012aa1n03x5               g129(.a(new_n216), .b(new_n224), .c(new_n219), .o1(new_n225));
  norp02aa1n03x5               g130(.a(new_n225), .b(new_n218), .o1(\s[20] ));
  nona23aa1d18x5               g131(.a(new_n217), .b(new_n197), .c(new_n203), .d(new_n222), .out0(new_n227));
  oai022aa1d18x5               g132(.a(\a[19] ), .b(\b[18] ), .c(\b[19] ), .d(\a[20] ), .o1(new_n228));
  nanp02aa1n02x5               g133(.a(\b[17] ), .b(\a[18] ), .o1(new_n229));
  oai022aa1n02x5               g134(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n230));
  nand03aa1n02x5               g135(.a(new_n230), .b(new_n229), .c(new_n215), .o1(new_n231));
  aboi22aa1n03x5               g136(.a(new_n228), .b(new_n231), .c(\a[20] ), .d(\b[19] ), .out0(new_n232));
  inv000aa1n02x5               g137(.a(new_n232), .o1(new_n233));
  aoai13aa1n06x5               g138(.a(new_n233), .b(new_n227), .c(new_n190), .d(new_n196), .o1(new_n234));
  xorb03aa1n02x5               g139(.a(new_n234), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  inv000aa1d42x5               g140(.a(\a[21] ), .o1(new_n236));
  nanb02aa1d24x5               g141(.a(\b[20] ), .b(new_n236), .out0(new_n237));
  inv000aa1d42x5               g142(.a(new_n237), .o1(new_n238));
  xorc02aa1n02x5               g143(.a(\a[21] ), .b(\b[20] ), .out0(new_n239));
  xnrc02aa1n12x5               g144(.a(\b[21] ), .b(\a[22] ), .out0(new_n240));
  inv000aa1d42x5               g145(.a(new_n240), .o1(new_n241));
  aoi112aa1n03x4               g146(.a(new_n238), .b(new_n241), .c(new_n234), .d(new_n239), .o1(new_n242));
  inv000aa1n02x5               g147(.a(new_n227), .o1(new_n243));
  aoai13aa1n03x5               g148(.a(new_n239), .b(new_n232), .c(new_n221), .d(new_n243), .o1(new_n244));
  tech160nm_fiaoi012aa1n02p5x5 g149(.a(new_n240), .b(new_n244), .c(new_n237), .o1(new_n245));
  nor002aa1n02x5               g150(.a(new_n245), .b(new_n242), .o1(\s[22] ));
  nand42aa1n02x5               g151(.a(\b[20] ), .b(\a[21] ), .o1(new_n247));
  nano22aa1n12x5               g152(.a(new_n240), .b(new_n237), .c(new_n247), .out0(new_n248));
  nona23aa1d18x5               g153(.a(new_n207), .b(new_n248), .c(new_n216), .d(new_n222), .out0(new_n249));
  nanb02aa1n06x5               g154(.a(new_n228), .b(new_n231), .out0(new_n250));
  nand02aa1n02x5               g155(.a(\b[19] ), .b(\a[20] ), .o1(new_n251));
  nano32aa1d12x5               g156(.a(new_n240), .b(new_n247), .c(new_n237), .d(new_n251), .out0(new_n252));
  oao003aa1n03x5               g157(.a(\a[22] ), .b(\b[21] ), .c(new_n237), .carry(new_n253));
  inv040aa1n03x5               g158(.a(new_n253), .o1(new_n254));
  aoi012aa1d24x5               g159(.a(new_n254), .b(new_n250), .c(new_n252), .o1(new_n255));
  aoai13aa1n06x5               g160(.a(new_n255), .b(new_n249), .c(new_n190), .d(new_n196), .o1(new_n256));
  xorb03aa1n02x5               g161(.a(new_n256), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  nor042aa1n03x5               g162(.a(\b[22] ), .b(\a[23] ), .o1(new_n258));
  tech160nm_fixorc02aa1n04x5   g163(.a(\a[23] ), .b(\b[22] ), .out0(new_n259));
  xorc02aa1n12x5               g164(.a(\a[24] ), .b(\b[23] ), .out0(new_n260));
  aoi112aa1n03x4               g165(.a(new_n258), .b(new_n260), .c(new_n256), .d(new_n259), .o1(new_n261));
  inv000aa1n02x5               g166(.a(new_n258), .o1(new_n262));
  inv040aa1n06x5               g167(.a(new_n249), .o1(new_n263));
  inv000aa1n02x5               g168(.a(new_n255), .o1(new_n264));
  aoai13aa1n03x5               g169(.a(new_n259), .b(new_n264), .c(new_n221), .d(new_n263), .o1(new_n265));
  inv000aa1d42x5               g170(.a(new_n260), .o1(new_n266));
  aoi012aa1n03x5               g171(.a(new_n266), .b(new_n265), .c(new_n262), .o1(new_n267));
  nor002aa1n02x5               g172(.a(new_n267), .b(new_n261), .o1(\s[24] ));
  nano32aa1n03x7               g173(.a(new_n227), .b(new_n260), .c(new_n248), .d(new_n259), .out0(new_n269));
  inv000aa1n02x5               g174(.a(new_n269), .o1(new_n270));
  oab012aa1n02x5               g175(.a(new_n199), .b(\a[18] ), .c(\b[17] ), .out0(new_n271));
  nano22aa1n02x4               g176(.a(new_n271), .b(new_n229), .c(new_n215), .out0(new_n272));
  oai012aa1n06x5               g177(.a(new_n252), .b(new_n272), .c(new_n228), .o1(new_n273));
  nand42aa1n02x5               g178(.a(new_n260), .b(new_n259), .o1(new_n274));
  oao003aa1n02x5               g179(.a(\a[24] ), .b(\b[23] ), .c(new_n262), .carry(new_n275));
  aoai13aa1n06x5               g180(.a(new_n275), .b(new_n274), .c(new_n273), .d(new_n253), .o1(new_n276));
  inv040aa1n03x5               g181(.a(new_n276), .o1(new_n277));
  aoai13aa1n06x5               g182(.a(new_n277), .b(new_n270), .c(new_n190), .d(new_n196), .o1(new_n278));
  xorb03aa1n02x5               g183(.a(new_n278), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  nor042aa1n03x5               g184(.a(\b[24] ), .b(\a[25] ), .o1(new_n280));
  tech160nm_fixorc02aa1n03p5x5 g185(.a(\a[25] ), .b(\b[24] ), .out0(new_n281));
  xorc02aa1n12x5               g186(.a(\a[26] ), .b(\b[25] ), .out0(new_n282));
  aoi112aa1n03x4               g187(.a(new_n280), .b(new_n282), .c(new_n278), .d(new_n281), .o1(new_n283));
  inv000aa1n02x5               g188(.a(new_n280), .o1(new_n284));
  aoai13aa1n02x5               g189(.a(new_n281), .b(new_n276), .c(new_n221), .d(new_n269), .o1(new_n285));
  inv000aa1d42x5               g190(.a(new_n282), .o1(new_n286));
  aoi012aa1n03x5               g191(.a(new_n286), .b(new_n285), .c(new_n284), .o1(new_n287));
  norp02aa1n03x5               g192(.a(new_n287), .b(new_n283), .o1(\s[26] ));
  inv000aa1n03x5               g193(.a(new_n274), .o1(new_n289));
  and002aa1n02x5               g194(.a(new_n282), .b(new_n281), .o(new_n290));
  nano22aa1n06x5               g195(.a(new_n249), .b(new_n289), .c(new_n290), .out0(new_n291));
  aoai13aa1n06x5               g196(.a(new_n291), .b(new_n201), .c(new_n124), .d(new_n189), .o1(new_n292));
  oao003aa1n02x5               g197(.a(\a[26] ), .b(\b[25] ), .c(new_n284), .carry(new_n293));
  aobi12aa1n06x5               g198(.a(new_n293), .b(new_n276), .c(new_n290), .out0(new_n294));
  xorc02aa1n12x5               g199(.a(\a[27] ), .b(\b[26] ), .out0(new_n295));
  xnbna2aa1n03x5               g200(.a(new_n295), .b(new_n294), .c(new_n292), .out0(\s[27] ));
  nor042aa1n03x5               g201(.a(\b[26] ), .b(\a[27] ), .o1(new_n297));
  inv040aa1n03x5               g202(.a(new_n297), .o1(new_n298));
  aobi12aa1n03x5               g203(.a(new_n295), .b(new_n294), .c(new_n292), .out0(new_n299));
  xnrc02aa1n12x5               g204(.a(\b[27] ), .b(\a[28] ), .out0(new_n300));
  nano22aa1n03x5               g205(.a(new_n299), .b(new_n298), .c(new_n300), .out0(new_n301));
  aoai13aa1n06x5               g206(.a(new_n289), .b(new_n254), .c(new_n250), .d(new_n252), .o1(new_n302));
  inv000aa1n02x5               g207(.a(new_n290), .o1(new_n303));
  aoai13aa1n06x5               g208(.a(new_n293), .b(new_n303), .c(new_n302), .d(new_n275), .o1(new_n304));
  aoai13aa1n02x5               g209(.a(new_n295), .b(new_n304), .c(new_n221), .d(new_n291), .o1(new_n305));
  aoi012aa1n02x5               g210(.a(new_n300), .b(new_n305), .c(new_n298), .o1(new_n306));
  norp02aa1n03x5               g211(.a(new_n306), .b(new_n301), .o1(\s[28] ));
  xnrc02aa1n02x5               g212(.a(\b[28] ), .b(\a[29] ), .out0(new_n308));
  norb02aa1n02x5               g213(.a(new_n295), .b(new_n300), .out0(new_n309));
  aoai13aa1n02x5               g214(.a(new_n309), .b(new_n304), .c(new_n221), .d(new_n291), .o1(new_n310));
  oao003aa1n02x5               g215(.a(\a[28] ), .b(\b[27] ), .c(new_n298), .carry(new_n311));
  aoi012aa1n02x5               g216(.a(new_n308), .b(new_n310), .c(new_n311), .o1(new_n312));
  aobi12aa1n03x5               g217(.a(new_n309), .b(new_n294), .c(new_n292), .out0(new_n313));
  nano22aa1n03x5               g218(.a(new_n313), .b(new_n308), .c(new_n311), .out0(new_n314));
  norp02aa1n03x5               g219(.a(new_n312), .b(new_n314), .o1(\s[29] ));
  xorb03aa1n02x5               g220(.a(new_n105), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g221(.a(new_n295), .b(new_n308), .c(new_n300), .out0(new_n317));
  aoai13aa1n02x5               g222(.a(new_n317), .b(new_n304), .c(new_n221), .d(new_n291), .o1(new_n318));
  oao003aa1n02x5               g223(.a(\a[29] ), .b(\b[28] ), .c(new_n311), .carry(new_n319));
  xnrc02aa1n02x5               g224(.a(\b[29] ), .b(\a[30] ), .out0(new_n320));
  aoi012aa1n02x5               g225(.a(new_n320), .b(new_n318), .c(new_n319), .o1(new_n321));
  aobi12aa1n03x5               g226(.a(new_n317), .b(new_n294), .c(new_n292), .out0(new_n322));
  nano22aa1n03x5               g227(.a(new_n322), .b(new_n319), .c(new_n320), .out0(new_n323));
  norp02aa1n03x5               g228(.a(new_n321), .b(new_n323), .o1(\s[30] ));
  xnrc02aa1n02x5               g229(.a(\b[30] ), .b(\a[31] ), .out0(new_n325));
  nona32aa1n02x4               g230(.a(new_n295), .b(new_n320), .c(new_n308), .d(new_n300), .out0(new_n326));
  aoi012aa1n03x5               g231(.a(new_n326), .b(new_n294), .c(new_n292), .o1(new_n327));
  oao003aa1n02x5               g232(.a(\a[30] ), .b(\b[29] ), .c(new_n319), .carry(new_n328));
  nano22aa1n03x5               g233(.a(new_n327), .b(new_n325), .c(new_n328), .out0(new_n329));
  inv000aa1n02x5               g234(.a(new_n326), .o1(new_n330));
  aoai13aa1n02x5               g235(.a(new_n330), .b(new_n304), .c(new_n221), .d(new_n291), .o1(new_n331));
  aoi012aa1n02x5               g236(.a(new_n325), .b(new_n331), .c(new_n328), .o1(new_n332));
  norp02aa1n03x5               g237(.a(new_n332), .b(new_n329), .o1(\s[31] ));
  xnbna2aa1n03x5               g238(.a(new_n142), .b(new_n109), .c(new_n99), .out0(\s[3] ));
  oaoi03aa1n02x5               g239(.a(\a[3] ), .b(\b[2] ), .c(new_n142), .o1(new_n335));
  xorb03aa1n02x5               g240(.a(new_n335), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g241(.a(new_n164), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  aoai13aa1n02x5               g242(.a(new_n120), .b(new_n146), .c(new_n111), .d(new_n101), .o1(new_n338));
  xorb03aa1n02x5               g243(.a(new_n338), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  norp02aa1n02x5               g244(.a(new_n146), .b(new_n145), .o1(new_n340));
  tech160nm_fiao0012aa1n02p5x5 g245(.a(new_n121), .b(new_n164), .c(new_n340), .o(new_n341));
  xorb03aa1n02x5               g246(.a(new_n341), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  aoi012aa1n02x5               g247(.a(new_n117), .b(new_n341), .c(new_n114), .o1(new_n343));
  xnrb03aa1n03x5               g248(.a(new_n343), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xorb03aa1n02x5               g249(.a(new_n124), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


