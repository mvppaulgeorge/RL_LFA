// Benchmark "adder" written by ABC on Thu Jul 18 10:14:27 2024

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
    new_n132, new_n133, new_n134, new_n135, new_n136, new_n137, new_n139,
    new_n140, new_n141, new_n142, new_n144, new_n145, new_n146, new_n147,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n159, new_n160, new_n161, new_n163, new_n164,
    new_n165, new_n166, new_n167, new_n168, new_n170, new_n171, new_n172,
    new_n173, new_n174, new_n175, new_n176, new_n177, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n186, new_n187,
    new_n188, new_n189, new_n190, new_n191, new_n192, new_n194, new_n195,
    new_n196, new_n197, new_n198, new_n199, new_n200, new_n201, new_n203,
    new_n204, new_n205, new_n206, new_n207, new_n210, new_n211, new_n212,
    new_n213, new_n214, new_n215, new_n216, new_n218, new_n219, new_n220,
    new_n221, new_n222, new_n223, new_n224, new_n226, new_n227, new_n228,
    new_n229, new_n230, new_n232, new_n233, new_n234, new_n235, new_n236,
    new_n237, new_n238, new_n239, new_n240, new_n241, new_n242, new_n243,
    new_n245, new_n246, new_n247, new_n248, new_n249, new_n251, new_n252,
    new_n253, new_n254, new_n255, new_n256, new_n257, new_n258, new_n260,
    new_n261, new_n262, new_n263, new_n264, new_n266, new_n267, new_n268,
    new_n269, new_n270, new_n271, new_n272, new_n274, new_n275, new_n276,
    new_n277, new_n278, new_n279, new_n280, new_n281, new_n282, new_n283,
    new_n284, new_n285, new_n286, new_n288, new_n289, new_n290, new_n291,
    new_n292, new_n293, new_n294, new_n297, new_n298, new_n299, new_n300,
    new_n301, new_n302, new_n303, new_n305, new_n306, new_n307, new_n308,
    new_n309, new_n310, new_n311, new_n314, new_n317, new_n318, new_n319,
    new_n322;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  xorc02aa1n12x5               g001(.a(\a[10] ), .b(\b[9] ), .out0(new_n97));
  inv000aa1d42x5               g002(.a(new_n97), .o1(new_n98));
  nor042aa1n03x5               g003(.a(\b[8] ), .b(\a[9] ), .o1(new_n99));
  aoi112aa1n02x5               g004(.a(\b[6] ), .b(\a[7] ), .c(\a[8] ), .d(\b[7] ), .o1(new_n100));
  norp02aa1n02x5               g005(.a(\b[7] ), .b(\a[8] ), .o1(new_n101));
  oai022aa1n02x7               g006(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n102));
  xnrc02aa1n02x5               g007(.a(\b[7] ), .b(\a[8] ), .out0(new_n103));
  nor042aa1n04x5               g008(.a(\b[6] ), .b(\a[7] ), .o1(new_n104));
  nand42aa1n16x5               g009(.a(\b[6] ), .b(\a[7] ), .o1(new_n105));
  nanb02aa1n06x5               g010(.a(new_n104), .b(new_n105), .out0(new_n106));
  and002aa1n12x5               g011(.a(\b[5] ), .b(\a[6] ), .o(new_n107));
  inv040aa1n03x5               g012(.a(new_n107), .o1(new_n108));
  nano23aa1n02x4               g013(.a(new_n103), .b(new_n106), .c(new_n108), .d(new_n102), .out0(new_n109));
  nor043aa1n02x5               g014(.a(new_n109), .b(new_n101), .c(new_n100), .o1(new_n110));
  nor002aa1n03x5               g015(.a(\b[1] ), .b(\a[2] ), .o1(new_n111));
  nand02aa1d04x5               g016(.a(\b[0] ), .b(\a[1] ), .o1(new_n112));
  nand02aa1d04x5               g017(.a(\b[1] ), .b(\a[2] ), .o1(new_n113));
  aoi012aa1n12x5               g018(.a(new_n111), .b(new_n112), .c(new_n113), .o1(new_n114));
  nor022aa1n06x5               g019(.a(\b[3] ), .b(\a[4] ), .o1(new_n115));
  nand22aa1n12x5               g020(.a(\b[3] ), .b(\a[4] ), .o1(new_n116));
  nor002aa1d24x5               g021(.a(\b[2] ), .b(\a[3] ), .o1(new_n117));
  nand42aa1n03x5               g022(.a(\b[2] ), .b(\a[3] ), .o1(new_n118));
  nona23aa1n09x5               g023(.a(new_n118), .b(new_n116), .c(new_n115), .d(new_n117), .out0(new_n119));
  ao0012aa1n03x7               g024(.a(new_n115), .b(new_n117), .c(new_n116), .o(new_n120));
  oab012aa1n03x5               g025(.a(new_n120), .b(new_n119), .c(new_n114), .out0(new_n121));
  nor042aa1n04x5               g026(.a(\b[5] ), .b(\a[6] ), .o1(new_n122));
  nor042aa1n04x5               g027(.a(new_n107), .b(new_n122), .o1(new_n123));
  tech160nm_fixorc02aa1n04x5   g028(.a(\a[5] ), .b(\b[4] ), .out0(new_n124));
  nona23aa1n02x5               g029(.a(new_n124), .b(new_n123), .c(new_n103), .d(new_n106), .out0(new_n125));
  oai012aa1n04x7               g030(.a(new_n110), .b(new_n121), .c(new_n125), .o1(new_n126));
  tech160nm_fixorc02aa1n03p5x5 g031(.a(\a[9] ), .b(\b[8] ), .out0(new_n127));
  aoai13aa1n02x5               g032(.a(new_n98), .b(new_n99), .c(new_n126), .d(new_n127), .o1(new_n128));
  oab012aa1n02x4               g033(.a(new_n122), .b(\a[5] ), .c(\b[4] ), .out0(new_n129));
  xorc02aa1n03x5               g034(.a(\a[8] ), .b(\b[7] ), .out0(new_n130));
  norb02aa1n02x5               g035(.a(new_n105), .b(new_n104), .out0(new_n131));
  nona23aa1n03x5               g036(.a(new_n130), .b(new_n131), .c(new_n129), .d(new_n107), .out0(new_n132));
  nona22aa1n09x5               g037(.a(new_n132), .b(new_n101), .c(new_n100), .out0(new_n133));
  oabi12aa1n18x5               g038(.a(new_n120), .b(new_n119), .c(new_n114), .out0(new_n134));
  nano32aa1n03x7               g039(.a(new_n103), .b(new_n124), .c(new_n131), .d(new_n123), .out0(new_n135));
  aoai13aa1n03x5               g040(.a(new_n127), .b(new_n133), .c(new_n135), .d(new_n134), .o1(new_n136));
  nona22aa1n03x5               g041(.a(new_n136), .b(new_n99), .c(new_n98), .out0(new_n137));
  nanp02aa1n02x5               g042(.a(new_n128), .b(new_n137), .o1(\s[10] ));
  nand02aa1d06x5               g043(.a(\b[9] ), .b(\a[10] ), .o1(new_n139));
  nor002aa1n06x5               g044(.a(\b[10] ), .b(\a[11] ), .o1(new_n140));
  nand42aa1n08x5               g045(.a(\b[10] ), .b(\a[11] ), .o1(new_n141));
  nanb02aa1n06x5               g046(.a(new_n140), .b(new_n141), .out0(new_n142));
  xnbna2aa1n03x5               g047(.a(new_n142), .b(new_n137), .c(new_n139), .out0(\s[11] ));
  aoi013aa1n03x5               g048(.a(new_n140), .b(new_n137), .c(new_n139), .d(new_n141), .o1(new_n144));
  nor042aa1n04x5               g049(.a(\b[11] ), .b(\a[12] ), .o1(new_n145));
  nanp02aa1n04x5               g050(.a(\b[11] ), .b(\a[12] ), .o1(new_n146));
  norb02aa1n06x4               g051(.a(new_n146), .b(new_n145), .out0(new_n147));
  xnrc02aa1n03x5               g052(.a(new_n144), .b(new_n147), .out0(\s[12] ));
  nano23aa1n03x7               g053(.a(new_n140), .b(new_n145), .c(new_n146), .d(new_n141), .out0(new_n149));
  and003aa1n02x5               g054(.a(new_n149), .b(new_n127), .c(new_n97), .o(new_n150));
  aoai13aa1n06x5               g055(.a(new_n150), .b(new_n133), .c(new_n135), .d(new_n134), .o1(new_n151));
  aoi112aa1n03x5               g056(.a(\b[10] ), .b(\a[11] ), .c(\a[12] ), .d(\b[11] ), .o1(new_n152));
  nanb02aa1n03x5               g057(.a(new_n145), .b(new_n146), .out0(new_n153));
  oai022aa1n02x5               g058(.a(\a[10] ), .b(\b[9] ), .c(\b[8] ), .d(\a[9] ), .o1(new_n154));
  nano23aa1n06x5               g059(.a(new_n153), .b(new_n142), .c(new_n154), .d(new_n139), .out0(new_n155));
  nor043aa1n03x5               g060(.a(new_n155), .b(new_n152), .c(new_n145), .o1(new_n156));
  nanp02aa1n03x5               g061(.a(new_n151), .b(new_n156), .o1(new_n157));
  xorb03aa1n02x5               g062(.a(new_n157), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  nor042aa1n06x5               g063(.a(\b[12] ), .b(\a[13] ), .o1(new_n159));
  nand42aa1d28x5               g064(.a(\b[12] ), .b(\a[13] ), .o1(new_n160));
  tech160nm_fiaoi012aa1n05x5   g065(.a(new_n159), .b(new_n157), .c(new_n160), .o1(new_n161));
  xnrb03aa1n03x5               g066(.a(new_n161), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  nor042aa1n09x5               g067(.a(\b[13] ), .b(\a[14] ), .o1(new_n163));
  nanp02aa1n24x5               g068(.a(\b[13] ), .b(\a[14] ), .o1(new_n164));
  nano23aa1d15x5               g069(.a(new_n159), .b(new_n163), .c(new_n164), .d(new_n160), .out0(new_n165));
  inv000aa1d42x5               g070(.a(new_n165), .o1(new_n166));
  tech160nm_fioai012aa1n03p5x5 g071(.a(new_n164), .b(new_n163), .c(new_n159), .o1(new_n167));
  aoai13aa1n04x5               g072(.a(new_n167), .b(new_n166), .c(new_n151), .d(new_n156), .o1(new_n168));
  xorb03aa1n02x5               g073(.a(new_n168), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  nor042aa1n06x5               g074(.a(\b[14] ), .b(\a[15] ), .o1(new_n170));
  nand22aa1n03x5               g075(.a(\b[14] ), .b(\a[15] ), .o1(new_n171));
  norb02aa1n09x5               g076(.a(new_n171), .b(new_n170), .out0(new_n172));
  nor042aa1n06x5               g077(.a(\b[15] ), .b(\a[16] ), .o1(new_n173));
  nanp02aa1n03x5               g078(.a(\b[15] ), .b(\a[16] ), .o1(new_n174));
  norb02aa1n06x5               g079(.a(new_n174), .b(new_n173), .out0(new_n175));
  aoi112aa1n02x5               g080(.a(new_n175), .b(new_n170), .c(new_n168), .d(new_n172), .o1(new_n176));
  aoai13aa1n02x5               g081(.a(new_n175), .b(new_n170), .c(new_n168), .d(new_n171), .o1(new_n177));
  norb02aa1n03x4               g082(.a(new_n177), .b(new_n176), .out0(\s[16] ));
  inv000aa1n02x5               g083(.a(new_n139), .o1(new_n179));
  norb02aa1n02x5               g084(.a(new_n141), .b(new_n140), .out0(new_n180));
  oab012aa1n03x5               g085(.a(new_n99), .b(\a[10] ), .c(\b[9] ), .out0(new_n181));
  nona23aa1n03x5               g086(.a(new_n147), .b(new_n180), .c(new_n181), .d(new_n179), .out0(new_n182));
  nona22aa1n03x5               g087(.a(new_n182), .b(new_n152), .c(new_n145), .out0(new_n183));
  nand23aa1d12x5               g088(.a(new_n165), .b(new_n172), .c(new_n175), .o1(new_n184));
  inv000aa1n02x5               g089(.a(new_n184), .o1(new_n185));
  nona23aa1n03x5               g090(.a(new_n174), .b(new_n171), .c(new_n170), .d(new_n173), .out0(new_n186));
  nanp02aa1n02x5               g091(.a(new_n170), .b(new_n174), .o1(new_n187));
  oai122aa1n06x5               g092(.a(new_n187), .b(new_n186), .c(new_n167), .d(\b[15] ), .e(\a[16] ), .o1(new_n188));
  aoi012aa1n12x5               g093(.a(new_n188), .b(new_n183), .c(new_n185), .o1(new_n189));
  nano32aa1d12x5               g094(.a(new_n184), .b(new_n149), .c(new_n127), .d(new_n97), .out0(new_n190));
  aoai13aa1n12x5               g095(.a(new_n190), .b(new_n133), .c(new_n135), .d(new_n134), .o1(new_n191));
  xorc02aa1n12x5               g096(.a(\a[17] ), .b(\b[16] ), .out0(new_n192));
  xnbna2aa1n03x5               g097(.a(new_n192), .b(new_n191), .c(new_n189), .out0(\s[17] ));
  inv040aa1d32x5               g098(.a(\a[17] ), .o1(new_n194));
  inv040aa1n12x5               g099(.a(\b[16] ), .o1(new_n195));
  nand22aa1n03x5               g100(.a(new_n195), .b(new_n194), .o1(new_n196));
  oabi12aa1n03x5               g101(.a(new_n188), .b(new_n156), .c(new_n184), .out0(new_n197));
  aoai13aa1n03x5               g102(.a(new_n192), .b(new_n197), .c(new_n126), .d(new_n190), .o1(new_n198));
  nor002aa1d32x5               g103(.a(\b[17] ), .b(\a[18] ), .o1(new_n199));
  nand02aa1d20x5               g104(.a(\b[17] ), .b(\a[18] ), .o1(new_n200));
  norb02aa1n02x5               g105(.a(new_n200), .b(new_n199), .out0(new_n201));
  xnbna2aa1n03x5               g106(.a(new_n201), .b(new_n198), .c(new_n196), .out0(\s[18] ));
  nand22aa1n03x5               g107(.a(\b[16] ), .b(\a[17] ), .o1(new_n203));
  nano32aa1d12x5               g108(.a(new_n199), .b(new_n196), .c(new_n200), .d(new_n203), .out0(new_n204));
  inv000aa1d42x5               g109(.a(new_n204), .o1(new_n205));
  aoai13aa1n12x5               g110(.a(new_n200), .b(new_n199), .c(new_n194), .d(new_n195), .o1(new_n206));
  aoai13aa1n04x5               g111(.a(new_n206), .b(new_n205), .c(new_n191), .d(new_n189), .o1(new_n207));
  xorb03aa1n02x5               g112(.a(new_n207), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g113(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor002aa1n20x5               g114(.a(\b[18] ), .b(\a[19] ), .o1(new_n210));
  nand22aa1n12x5               g115(.a(\b[18] ), .b(\a[19] ), .o1(new_n211));
  nor002aa1d32x5               g116(.a(\b[19] ), .b(\a[20] ), .o1(new_n212));
  nand22aa1n12x5               g117(.a(\b[19] ), .b(\a[20] ), .o1(new_n213));
  norb02aa1n02x5               g118(.a(new_n213), .b(new_n212), .out0(new_n214));
  aoi112aa1n03x4               g119(.a(new_n210), .b(new_n214), .c(new_n207), .d(new_n211), .o1(new_n215));
  aoai13aa1n03x5               g120(.a(new_n214), .b(new_n210), .c(new_n207), .d(new_n211), .o1(new_n216));
  norb02aa1n02x7               g121(.a(new_n216), .b(new_n215), .out0(\s[20] ));
  nano23aa1n06x5               g122(.a(new_n210), .b(new_n212), .c(new_n213), .d(new_n211), .out0(new_n218));
  nanp03aa1n02x5               g123(.a(new_n218), .b(new_n192), .c(new_n201), .o1(new_n219));
  nona23aa1n09x5               g124(.a(new_n213), .b(new_n211), .c(new_n210), .d(new_n212), .out0(new_n220));
  aoi012aa1n06x5               g125(.a(new_n212), .b(new_n210), .c(new_n213), .o1(new_n221));
  oai012aa1n12x5               g126(.a(new_n221), .b(new_n220), .c(new_n206), .o1(new_n222));
  inv000aa1d42x5               g127(.a(new_n222), .o1(new_n223));
  aoai13aa1n04x5               g128(.a(new_n223), .b(new_n219), .c(new_n191), .d(new_n189), .o1(new_n224));
  xorb03aa1n02x5               g129(.a(new_n224), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor042aa1n03x5               g130(.a(\b[20] ), .b(\a[21] ), .o1(new_n226));
  xorc02aa1n02x5               g131(.a(\a[21] ), .b(\b[20] ), .out0(new_n227));
  xorc02aa1n02x5               g132(.a(\a[22] ), .b(\b[21] ), .out0(new_n228));
  aoi112aa1n02x5               g133(.a(new_n226), .b(new_n228), .c(new_n224), .d(new_n227), .o1(new_n229));
  aoai13aa1n03x5               g134(.a(new_n228), .b(new_n226), .c(new_n224), .d(new_n227), .o1(new_n230));
  norb02aa1n02x7               g135(.a(new_n230), .b(new_n229), .out0(\s[22] ));
  inv000aa1d42x5               g136(.a(\a[21] ), .o1(new_n232));
  inv040aa1d32x5               g137(.a(\a[22] ), .o1(new_n233));
  xroi22aa1d06x4               g138(.a(new_n232), .b(\b[20] ), .c(new_n233), .d(\b[21] ), .out0(new_n234));
  nand23aa1n03x5               g139(.a(new_n234), .b(new_n204), .c(new_n218), .o1(new_n235));
  inv040aa1n02x5               g140(.a(new_n206), .o1(new_n236));
  inv000aa1n02x5               g141(.a(new_n221), .o1(new_n237));
  aoai13aa1n06x5               g142(.a(new_n234), .b(new_n237), .c(new_n218), .d(new_n236), .o1(new_n238));
  inv000aa1d42x5               g143(.a(\b[21] ), .o1(new_n239));
  oaoi03aa1n09x5               g144(.a(new_n233), .b(new_n239), .c(new_n226), .o1(new_n240));
  nanp02aa1n03x5               g145(.a(new_n238), .b(new_n240), .o1(new_n241));
  inv000aa1d42x5               g146(.a(new_n241), .o1(new_n242));
  aoai13aa1n04x5               g147(.a(new_n242), .b(new_n235), .c(new_n191), .d(new_n189), .o1(new_n243));
  xorb03aa1n02x5               g148(.a(new_n243), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g149(.a(\b[22] ), .b(\a[23] ), .o1(new_n245));
  tech160nm_fixorc02aa1n05x5   g150(.a(\a[23] ), .b(\b[22] ), .out0(new_n246));
  xorc02aa1n03x5               g151(.a(\a[24] ), .b(\b[23] ), .out0(new_n247));
  aoi112aa1n02x5               g152(.a(new_n245), .b(new_n247), .c(new_n243), .d(new_n246), .o1(new_n248));
  aoai13aa1n03x5               g153(.a(new_n247), .b(new_n245), .c(new_n243), .d(new_n246), .o1(new_n249));
  norb02aa1n02x7               g154(.a(new_n249), .b(new_n248), .out0(\s[24] ));
  and002aa1n02x7               g155(.a(new_n247), .b(new_n246), .o(new_n251));
  nanb03aa1n02x5               g156(.a(new_n219), .b(new_n251), .c(new_n234), .out0(new_n252));
  inv000aa1n02x5               g157(.a(new_n251), .o1(new_n253));
  orn002aa1n02x5               g158(.a(\a[23] ), .b(\b[22] ), .o(new_n254));
  oao003aa1n02x5               g159(.a(\a[24] ), .b(\b[23] ), .c(new_n254), .carry(new_n255));
  aoai13aa1n02x5               g160(.a(new_n255), .b(new_n253), .c(new_n238), .d(new_n240), .o1(new_n256));
  inv040aa1n03x5               g161(.a(new_n256), .o1(new_n257));
  aoai13aa1n04x5               g162(.a(new_n257), .b(new_n252), .c(new_n191), .d(new_n189), .o1(new_n258));
  xorb03aa1n02x5               g163(.a(new_n258), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g164(.a(\b[24] ), .b(\a[25] ), .o1(new_n260));
  xorc02aa1n12x5               g165(.a(\a[25] ), .b(\b[24] ), .out0(new_n261));
  xorc02aa1n12x5               g166(.a(\a[26] ), .b(\b[25] ), .out0(new_n262));
  aoi112aa1n03x4               g167(.a(new_n260), .b(new_n262), .c(new_n258), .d(new_n261), .o1(new_n263));
  aoai13aa1n03x5               g168(.a(new_n262), .b(new_n260), .c(new_n258), .d(new_n261), .o1(new_n264));
  norb02aa1n02x7               g169(.a(new_n264), .b(new_n263), .out0(\s[26] ));
  and002aa1n12x5               g170(.a(new_n262), .b(new_n261), .o(new_n266));
  nano22aa1n03x7               g171(.a(new_n235), .b(new_n251), .c(new_n266), .out0(new_n267));
  aoai13aa1n06x5               g172(.a(new_n267), .b(new_n197), .c(new_n126), .d(new_n190), .o1(new_n268));
  orn002aa1n02x5               g173(.a(\a[25] ), .b(\b[24] ), .o(new_n269));
  oao003aa1n02x5               g174(.a(\a[26] ), .b(\b[25] ), .c(new_n269), .carry(new_n270));
  aobi12aa1n06x5               g175(.a(new_n270), .b(new_n256), .c(new_n266), .out0(new_n271));
  xorc02aa1n12x5               g176(.a(\a[27] ), .b(\b[26] ), .out0(new_n272));
  xnbna2aa1n03x5               g177(.a(new_n272), .b(new_n271), .c(new_n268), .out0(\s[27] ));
  nor042aa1n03x5               g178(.a(\b[26] ), .b(\a[27] ), .o1(new_n274));
  inv040aa1n03x5               g179(.a(new_n274), .o1(new_n275));
  aobi12aa1n02x7               g180(.a(new_n272), .b(new_n271), .c(new_n268), .out0(new_n276));
  xnrc02aa1n02x5               g181(.a(\b[27] ), .b(\a[28] ), .out0(new_n277));
  nano22aa1n03x5               g182(.a(new_n276), .b(new_n275), .c(new_n277), .out0(new_n278));
  inv000aa1n02x5               g183(.a(new_n267), .o1(new_n279));
  tech160nm_fiaoi012aa1n05x5   g184(.a(new_n279), .b(new_n191), .c(new_n189), .o1(new_n280));
  inv000aa1n02x5               g185(.a(new_n240), .o1(new_n281));
  aoai13aa1n02x7               g186(.a(new_n251), .b(new_n281), .c(new_n222), .d(new_n234), .o1(new_n282));
  inv000aa1d42x5               g187(.a(new_n266), .o1(new_n283));
  aoai13aa1n06x5               g188(.a(new_n270), .b(new_n283), .c(new_n282), .d(new_n255), .o1(new_n284));
  oaih12aa1n02x5               g189(.a(new_n272), .b(new_n284), .c(new_n280), .o1(new_n285));
  tech160nm_fiaoi012aa1n03p5x5 g190(.a(new_n277), .b(new_n285), .c(new_n275), .o1(new_n286));
  norp02aa1n03x5               g191(.a(new_n286), .b(new_n278), .o1(\s[28] ));
  norb02aa1n02x5               g192(.a(new_n272), .b(new_n277), .out0(new_n288));
  oaih12aa1n02x5               g193(.a(new_n288), .b(new_n284), .c(new_n280), .o1(new_n289));
  oao003aa1n03x5               g194(.a(\a[28] ), .b(\b[27] ), .c(new_n275), .carry(new_n290));
  xnrc02aa1n02x5               g195(.a(\b[28] ), .b(\a[29] ), .out0(new_n291));
  tech160nm_fiaoi012aa1n03p5x5 g196(.a(new_n291), .b(new_n289), .c(new_n290), .o1(new_n292));
  aobi12aa1n02x7               g197(.a(new_n288), .b(new_n271), .c(new_n268), .out0(new_n293));
  nano22aa1n02x4               g198(.a(new_n293), .b(new_n290), .c(new_n291), .out0(new_n294));
  norp02aa1n03x5               g199(.a(new_n292), .b(new_n294), .o1(\s[29] ));
  xorb03aa1n02x5               g200(.a(new_n112), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g201(.a(new_n272), .b(new_n291), .c(new_n277), .out0(new_n297));
  oaih12aa1n02x5               g202(.a(new_n297), .b(new_n284), .c(new_n280), .o1(new_n298));
  oao003aa1n02x5               g203(.a(\a[29] ), .b(\b[28] ), .c(new_n290), .carry(new_n299));
  xnrc02aa1n02x5               g204(.a(\b[29] ), .b(\a[30] ), .out0(new_n300));
  tech160nm_fiaoi012aa1n03p5x5 g205(.a(new_n300), .b(new_n298), .c(new_n299), .o1(new_n301));
  aobi12aa1n02x7               g206(.a(new_n297), .b(new_n271), .c(new_n268), .out0(new_n302));
  nano22aa1n02x4               g207(.a(new_n302), .b(new_n299), .c(new_n300), .out0(new_n303));
  norp02aa1n03x5               g208(.a(new_n301), .b(new_n303), .o1(\s[30] ));
  xnrc02aa1n02x5               g209(.a(\b[30] ), .b(\a[31] ), .out0(new_n305));
  norb02aa1n02x5               g210(.a(new_n297), .b(new_n300), .out0(new_n306));
  aobi12aa1n02x7               g211(.a(new_n306), .b(new_n271), .c(new_n268), .out0(new_n307));
  oao003aa1n02x5               g212(.a(\a[30] ), .b(\b[29] ), .c(new_n299), .carry(new_n308));
  nano22aa1n02x4               g213(.a(new_n307), .b(new_n305), .c(new_n308), .out0(new_n309));
  oaih12aa1n02x5               g214(.a(new_n306), .b(new_n284), .c(new_n280), .o1(new_n310));
  tech160nm_fiaoi012aa1n03p5x5 g215(.a(new_n305), .b(new_n310), .c(new_n308), .o1(new_n311));
  norp02aa1n03x5               g216(.a(new_n311), .b(new_n309), .o1(\s[31] ));
  xnrb03aa1n02x5               g217(.a(new_n114), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g218(.a(\a[3] ), .b(\b[2] ), .c(new_n114), .o1(new_n314));
  xorb03aa1n02x5               g219(.a(new_n314), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g220(.a(new_n134), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  nanp02aa1n03x5               g221(.a(new_n134), .b(new_n124), .o1(new_n317));
  oaoi13aa1n02x5               g222(.a(new_n123), .b(new_n317), .c(\a[5] ), .d(\b[4] ), .o1(new_n318));
  oai112aa1n03x5               g223(.a(new_n317), .b(new_n123), .c(\b[4] ), .d(\a[5] ), .o1(new_n319));
  nanb02aa1n02x5               g224(.a(new_n318), .b(new_n319), .out0(\s[6] ));
  xnbna2aa1n03x5               g225(.a(new_n106), .b(new_n319), .c(new_n108), .out0(\s[7] ));
  aoi013aa1n03x5               g226(.a(new_n104), .b(new_n319), .c(new_n108), .d(new_n105), .o1(new_n322));
  xnrc02aa1n02x5               g227(.a(new_n322), .b(new_n130), .out0(\s[8] ));
  xorb03aa1n02x5               g228(.a(new_n126), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


