// Benchmark "adder" written by ABC on Wed Jul 10 17:23:17 2024

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
    new_n125, new_n126, new_n127, new_n128, new_n129, new_n131, new_n132,
    new_n133, new_n134, new_n135, new_n136, new_n137, new_n138, new_n139,
    new_n141, new_n142, new_n143, new_n144, new_n145, new_n146, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n158, new_n159, new_n160, new_n162, new_n163, new_n164,
    new_n165, new_n166, new_n168, new_n169, new_n170, new_n171, new_n172,
    new_n173, new_n174, new_n175, new_n176, new_n178, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n186, new_n187,
    new_n188, new_n190, new_n191, new_n192, new_n193, new_n195, new_n196,
    new_n197, new_n198, new_n199, new_n200, new_n201, new_n204, new_n205,
    new_n206, new_n207, new_n208, new_n209, new_n210, new_n211, new_n212,
    new_n213, new_n214, new_n216, new_n217, new_n218, new_n219, new_n220,
    new_n221, new_n222, new_n223, new_n224, new_n226, new_n227, new_n228,
    new_n229, new_n230, new_n231, new_n232, new_n233, new_n234, new_n236,
    new_n237, new_n238, new_n239, new_n240, new_n242, new_n243, new_n244,
    new_n245, new_n246, new_n247, new_n248, new_n249, new_n250, new_n252,
    new_n253, new_n254, new_n255, new_n256, new_n257, new_n258, new_n259,
    new_n260, new_n261, new_n263, new_n264, new_n265, new_n266, new_n267,
    new_n268, new_n269, new_n270, new_n272, new_n273, new_n274, new_n275,
    new_n276, new_n277, new_n278, new_n279, new_n280, new_n281, new_n283,
    new_n284, new_n285, new_n286, new_n287, new_n288, new_n289, new_n290,
    new_n291, new_n292, new_n294, new_n295, new_n296, new_n297, new_n298,
    new_n299, new_n300, new_n303, new_n304, new_n305, new_n306, new_n307,
    new_n308, new_n309, new_n311, new_n312, new_n313, new_n314, new_n315,
    new_n316, new_n317, new_n319, new_n321, new_n324, new_n326, new_n327,
    new_n328, new_n329, new_n331, new_n332;
  160nm_ficinv00aa1n08x5       g000(.clk(\a[0] ), .clkout(\s[0] ));
  orn002aa1n02x5               g001(.a(\a[9] ), .b(\b[8] ), .o(new_n97));
  norp02aa1n02x5               g002(.a(\b[1] ), .b(\a[2] ), .o1(new_n98));
  nanp02aa1n02x5               g003(.a(\b[0] ), .b(\a[1] ), .o1(new_n99));
  nanp02aa1n02x5               g004(.a(\b[1] ), .b(\a[2] ), .o1(new_n100));
  aoi012aa1n02x5               g005(.a(new_n98), .b(new_n99), .c(new_n100), .o1(new_n101));
  norp02aa1n02x5               g006(.a(\b[3] ), .b(\a[4] ), .o1(new_n102));
  nanp02aa1n02x5               g007(.a(\b[3] ), .b(\a[4] ), .o1(new_n103));
  norp02aa1n02x5               g008(.a(\b[2] ), .b(\a[3] ), .o1(new_n104));
  nanp02aa1n02x5               g009(.a(\b[2] ), .b(\a[3] ), .o1(new_n105));
  nona23aa1n02x4               g010(.a(new_n105), .b(new_n103), .c(new_n102), .d(new_n104), .out0(new_n106));
  oai012aa1n02x5               g011(.a(new_n103), .b(new_n104), .c(new_n102), .o1(new_n107));
  oai012aa1n02x5               g012(.a(new_n107), .b(new_n106), .c(new_n101), .o1(new_n108));
  norp02aa1n02x5               g013(.a(\b[7] ), .b(\a[8] ), .o1(new_n109));
  nanp02aa1n02x5               g014(.a(\b[7] ), .b(\a[8] ), .o1(new_n110));
  norp02aa1n02x5               g015(.a(\b[6] ), .b(\a[7] ), .o1(new_n111));
  nanp02aa1n02x5               g016(.a(\b[6] ), .b(\a[7] ), .o1(new_n112));
  nona23aa1n02x4               g017(.a(new_n112), .b(new_n110), .c(new_n109), .d(new_n111), .out0(new_n113));
  xnrc02aa1n02x5               g018(.a(\b[5] ), .b(\a[6] ), .out0(new_n114));
  xnrc02aa1n02x5               g019(.a(\b[4] ), .b(\a[5] ), .out0(new_n115));
  norp03aa1n02x5               g020(.a(new_n113), .b(new_n114), .c(new_n115), .o1(new_n116));
  nanp02aa1n02x5               g021(.a(new_n108), .b(new_n116), .o1(new_n117));
  160nm_ficinv00aa1n08x5       g022(.clk(new_n109), .clkout(new_n118));
  aoi112aa1n02x5               g023(.a(\b[6] ), .b(\a[7] ), .c(\a[8] ), .d(\b[7] ), .o1(new_n119));
  160nm_ficinv00aa1n08x5       g024(.clk(new_n119), .clkout(new_n120));
  and002aa1n02x5               g025(.a(\b[5] ), .b(\a[6] ), .o(new_n121));
  160nm_ficinv00aa1n08x5       g026(.clk(\a[5] ), .clkout(new_n122));
  160nm_ficinv00aa1n08x5       g027(.clk(\b[4] ), .clkout(new_n123));
  norp02aa1n02x5               g028(.a(\b[5] ), .b(\a[6] ), .o1(new_n124));
  aoi012aa1n02x5               g029(.a(new_n124), .b(new_n122), .c(new_n123), .o1(new_n125));
  norp03aa1n02x5               g030(.a(new_n113), .b(new_n121), .c(new_n125), .o1(new_n126));
  nano22aa1n02x4               g031(.a(new_n126), .b(new_n118), .c(new_n120), .out0(new_n127));
  xnrc02aa1n02x5               g032(.a(\b[8] ), .b(\a[9] ), .out0(new_n128));
  aoai13aa1n02x5               g033(.a(new_n97), .b(new_n128), .c(new_n127), .d(new_n117), .o1(new_n129));
  xorb03aa1n02x5               g034(.a(new_n129), .b(\b[9] ), .c(\a[10] ), .out0(\s[10] ));
  norp02aa1n02x5               g035(.a(\b[9] ), .b(\a[10] ), .o1(new_n131));
  nano23aa1n02x4               g036(.a(new_n109), .b(new_n111), .c(new_n112), .d(new_n110), .out0(new_n132));
  nona22aa1n02x4               g037(.a(new_n132), .b(new_n121), .c(new_n125), .out0(new_n133));
  nona22aa1n02x4               g038(.a(new_n133), .b(new_n119), .c(new_n109), .out0(new_n134));
  xnrc02aa1n02x5               g039(.a(\b[9] ), .b(\a[10] ), .out0(new_n135));
  norp02aa1n02x5               g040(.a(new_n135), .b(new_n128), .o1(new_n136));
  aoai13aa1n02x5               g041(.a(new_n136), .b(new_n134), .c(new_n108), .d(new_n116), .o1(new_n137));
  aoi112aa1n02x5               g042(.a(\b[8] ), .b(\a[9] ), .c(\a[10] ), .d(\b[9] ), .o1(new_n138));
  nona22aa1n02x4               g043(.a(new_n137), .b(new_n138), .c(new_n131), .out0(new_n139));
  xorb03aa1n02x5               g044(.a(new_n139), .b(\b[10] ), .c(\a[11] ), .out0(\s[11] ));
  norp02aa1n02x5               g045(.a(\b[10] ), .b(\a[11] ), .o1(new_n141));
  nanp02aa1n02x5               g046(.a(\b[10] ), .b(\a[11] ), .o1(new_n142));
  aoi012aa1n02x5               g047(.a(new_n141), .b(new_n139), .c(new_n142), .o1(new_n143));
  norp02aa1n02x5               g048(.a(\b[11] ), .b(\a[12] ), .o1(new_n144));
  nanp02aa1n02x5               g049(.a(\b[11] ), .b(\a[12] ), .o1(new_n145));
  norb02aa1n02x5               g050(.a(new_n145), .b(new_n144), .out0(new_n146));
  xnrc02aa1n02x5               g051(.a(new_n143), .b(new_n146), .out0(\s[12] ));
  nano23aa1n02x4               g052(.a(new_n141), .b(new_n144), .c(new_n145), .d(new_n142), .out0(new_n148));
  norb03aa1n02x5               g053(.a(new_n148), .b(new_n128), .c(new_n135), .out0(new_n149));
  aoai13aa1n02x5               g054(.a(new_n149), .b(new_n134), .c(new_n108), .d(new_n116), .o1(new_n150));
  aoi112aa1n02x5               g055(.a(\b[10] ), .b(\a[11] ), .c(\a[12] ), .d(\b[11] ), .o1(new_n151));
  norb02aa1n02x5               g056(.a(new_n142), .b(new_n141), .out0(new_n152));
  oai112aa1n02x5               g057(.a(new_n146), .b(new_n152), .c(new_n138), .d(new_n131), .o1(new_n153));
  nona22aa1n02x4               g058(.a(new_n153), .b(new_n151), .c(new_n144), .out0(new_n154));
  160nm_ficinv00aa1n08x5       g059(.clk(new_n154), .clkout(new_n155));
  nanp02aa1n02x5               g060(.a(new_n150), .b(new_n155), .o1(new_n156));
  xorb03aa1n02x5               g061(.a(new_n156), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  norp02aa1n02x5               g062(.a(\b[12] ), .b(\a[13] ), .o1(new_n158));
  nanp02aa1n02x5               g063(.a(\b[12] ), .b(\a[13] ), .o1(new_n159));
  aoi012aa1n02x5               g064(.a(new_n158), .b(new_n156), .c(new_n159), .o1(new_n160));
  xnrb03aa1n02x5               g065(.a(new_n160), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  norp02aa1n02x5               g066(.a(\b[13] ), .b(\a[14] ), .o1(new_n162));
  nanp02aa1n02x5               g067(.a(\b[13] ), .b(\a[14] ), .o1(new_n163));
  nona23aa1n02x4               g068(.a(new_n163), .b(new_n159), .c(new_n158), .d(new_n162), .out0(new_n164));
  aoi012aa1n02x5               g069(.a(new_n162), .b(new_n158), .c(new_n163), .o1(new_n165));
  aoai13aa1n02x5               g070(.a(new_n165), .b(new_n164), .c(new_n150), .d(new_n155), .o1(new_n166));
  xorb03aa1n02x5               g071(.a(new_n166), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  norp02aa1n02x5               g072(.a(\b[14] ), .b(\a[15] ), .o1(new_n168));
  xnrc02aa1n02x5               g073(.a(\b[14] ), .b(\a[15] ), .out0(new_n169));
  160nm_ficinv00aa1n08x5       g074(.clk(new_n169), .clkout(new_n170));
  xnrc02aa1n02x5               g075(.a(\b[15] ), .b(\a[16] ), .out0(new_n171));
  160nm_ficinv00aa1n08x5       g076(.clk(new_n171), .clkout(new_n172));
  aoi112aa1n02x5               g077(.a(new_n172), .b(new_n168), .c(new_n166), .d(new_n170), .o1(new_n173));
  160nm_ficinv00aa1n08x5       g078(.clk(new_n168), .clkout(new_n174));
  nanp02aa1n02x5               g079(.a(new_n166), .b(new_n170), .o1(new_n175));
  aoi012aa1n02x5               g080(.a(new_n171), .b(new_n175), .c(new_n174), .o1(new_n176));
  norp02aa1n02x5               g081(.a(new_n176), .b(new_n173), .o1(\s[16] ));
  nano23aa1n02x4               g082(.a(new_n158), .b(new_n162), .c(new_n163), .d(new_n159), .out0(new_n178));
  nona22aa1n02x4               g083(.a(new_n178), .b(new_n171), .c(new_n169), .out0(new_n179));
  nano22aa1n02x4               g084(.a(new_n179), .b(new_n136), .c(new_n148), .out0(new_n180));
  aoai13aa1n02x5               g085(.a(new_n180), .b(new_n134), .c(new_n108), .d(new_n116), .o1(new_n181));
  norp03aa1n02x5               g086(.a(new_n164), .b(new_n171), .c(new_n169), .o1(new_n182));
  aoi112aa1n02x5               g087(.a(\b[14] ), .b(\a[15] ), .c(\a[16] ), .d(\b[15] ), .o1(new_n183));
  norp02aa1n02x5               g088(.a(\b[15] ), .b(\a[16] ), .o1(new_n184));
  160nm_ficinv00aa1n08x5       g089(.clk(new_n184), .clkout(new_n185));
  oai013aa1n02x4               g090(.a(new_n185), .b(new_n169), .c(new_n171), .d(new_n165), .o1(new_n186));
  aoi112aa1n02x5               g091(.a(new_n186), .b(new_n183), .c(new_n154), .d(new_n182), .o1(new_n187));
  xorc02aa1n02x5               g092(.a(\a[17] ), .b(\b[16] ), .out0(new_n188));
  xnbna2aa1n03x5               g093(.a(new_n188), .b(new_n181), .c(new_n187), .out0(\s[17] ));
  160nm_ficinv00aa1n08x5       g094(.clk(\a[18] ), .clkout(new_n190));
  nanp02aa1n02x5               g095(.a(new_n181), .b(new_n187), .o1(new_n191));
  norp02aa1n02x5               g096(.a(\b[16] ), .b(\a[17] ), .o1(new_n192));
  aoi012aa1n02x5               g097(.a(new_n192), .b(new_n191), .c(new_n188), .o1(new_n193));
  xorb03aa1n02x5               g098(.a(new_n193), .b(\b[17] ), .c(new_n190), .out0(\s[18] ));
  160nm_ficinv00aa1n08x5       g099(.clk(\a[17] ), .clkout(new_n195));
  xroi22aa1d04x5               g100(.a(new_n195), .b(\b[16] ), .c(new_n190), .d(\b[17] ), .out0(new_n196));
  160nm_ficinv00aa1n08x5       g101(.clk(new_n196), .clkout(new_n197));
  160nm_ficinv00aa1n08x5       g102(.clk(\b[17] ), .clkout(new_n198));
  oao003aa1n02x5               g103(.a(new_n190), .b(new_n198), .c(new_n192), .carry(new_n199));
  160nm_ficinv00aa1n08x5       g104(.clk(new_n199), .clkout(new_n200));
  aoai13aa1n02x5               g105(.a(new_n200), .b(new_n197), .c(new_n181), .d(new_n187), .o1(new_n201));
  xorb03aa1n02x5               g106(.a(new_n201), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g107(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  norp02aa1n02x5               g108(.a(\b[18] ), .b(\a[19] ), .o1(new_n204));
  nanp02aa1n02x5               g109(.a(\b[18] ), .b(\a[19] ), .o1(new_n205));
  norp02aa1n02x5               g110(.a(\b[19] ), .b(\a[20] ), .o1(new_n206));
  nanp02aa1n02x5               g111(.a(\b[19] ), .b(\a[20] ), .o1(new_n207));
  norb02aa1n02x5               g112(.a(new_n207), .b(new_n206), .out0(new_n208));
  aoi112aa1n02x5               g113(.a(new_n204), .b(new_n208), .c(new_n201), .d(new_n205), .o1(new_n209));
  160nm_ficinv00aa1n08x5       g114(.clk(new_n204), .clkout(new_n210));
  norb02aa1n02x5               g115(.a(new_n205), .b(new_n204), .out0(new_n211));
  nanp02aa1n02x5               g116(.a(new_n201), .b(new_n211), .o1(new_n212));
  160nm_ficinv00aa1n08x5       g117(.clk(new_n208), .clkout(new_n213));
  aoi012aa1n02x5               g118(.a(new_n213), .b(new_n212), .c(new_n210), .o1(new_n214));
  norp02aa1n02x5               g119(.a(new_n214), .b(new_n209), .o1(\s[20] ));
  nano23aa1n02x4               g120(.a(new_n204), .b(new_n206), .c(new_n207), .d(new_n205), .out0(new_n216));
  nanp02aa1n02x5               g121(.a(new_n196), .b(new_n216), .o1(new_n217));
  aoi112aa1n02x5               g122(.a(\b[18] ), .b(\a[19] ), .c(\a[20] ), .d(\b[19] ), .o1(new_n218));
  norp02aa1n02x5               g123(.a(\b[17] ), .b(\a[18] ), .o1(new_n219));
  aoi112aa1n02x5               g124(.a(\b[16] ), .b(\a[17] ), .c(\a[18] ), .d(\b[17] ), .o1(new_n220));
  oai112aa1n02x5               g125(.a(new_n211), .b(new_n208), .c(new_n220), .d(new_n219), .o1(new_n221));
  nona22aa1n02x4               g126(.a(new_n221), .b(new_n218), .c(new_n206), .out0(new_n222));
  160nm_ficinv00aa1n08x5       g127(.clk(new_n222), .clkout(new_n223));
  aoai13aa1n02x5               g128(.a(new_n223), .b(new_n217), .c(new_n181), .d(new_n187), .o1(new_n224));
  xorb03aa1n02x5               g129(.a(new_n224), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  norp02aa1n02x5               g130(.a(\b[20] ), .b(\a[21] ), .o1(new_n226));
  xnrc02aa1n02x5               g131(.a(\b[20] ), .b(\a[21] ), .out0(new_n227));
  160nm_ficinv00aa1n08x5       g132(.clk(new_n227), .clkout(new_n228));
  xnrc02aa1n02x5               g133(.a(\b[21] ), .b(\a[22] ), .out0(new_n229));
  160nm_ficinv00aa1n08x5       g134(.clk(new_n229), .clkout(new_n230));
  aoi112aa1n02x5               g135(.a(new_n226), .b(new_n230), .c(new_n224), .d(new_n228), .o1(new_n231));
  160nm_ficinv00aa1n08x5       g136(.clk(new_n226), .clkout(new_n232));
  nanp02aa1n02x5               g137(.a(new_n224), .b(new_n228), .o1(new_n233));
  aoi012aa1n02x5               g138(.a(new_n229), .b(new_n233), .c(new_n232), .o1(new_n234));
  norp02aa1n02x5               g139(.a(new_n234), .b(new_n231), .o1(\s[22] ));
  norp02aa1n02x5               g140(.a(new_n229), .b(new_n227), .o1(new_n236));
  nanp03aa1n02x5               g141(.a(new_n196), .b(new_n236), .c(new_n216), .o1(new_n237));
  oaoi03aa1n02x5               g142(.a(\a[22] ), .b(\b[21] ), .c(new_n232), .o1(new_n238));
  aoi012aa1n02x5               g143(.a(new_n238), .b(new_n222), .c(new_n236), .o1(new_n239));
  aoai13aa1n02x5               g144(.a(new_n239), .b(new_n237), .c(new_n181), .d(new_n187), .o1(new_n240));
  xorb03aa1n02x5               g145(.a(new_n240), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g146(.a(\b[22] ), .b(\a[23] ), .o1(new_n242));
  nanp02aa1n02x5               g147(.a(\b[22] ), .b(\a[23] ), .o1(new_n243));
  norb02aa1n02x5               g148(.a(new_n243), .b(new_n242), .out0(new_n244));
  xorc02aa1n02x5               g149(.a(\a[24] ), .b(\b[23] ), .out0(new_n245));
  aoi112aa1n02x5               g150(.a(new_n242), .b(new_n245), .c(new_n240), .d(new_n244), .o1(new_n246));
  160nm_ficinv00aa1n08x5       g151(.clk(new_n242), .clkout(new_n247));
  nanp02aa1n02x5               g152(.a(new_n240), .b(new_n244), .o1(new_n248));
  160nm_ficinv00aa1n08x5       g153(.clk(new_n245), .clkout(new_n249));
  aoi012aa1n02x5               g154(.a(new_n249), .b(new_n248), .c(new_n247), .o1(new_n250));
  norp02aa1n02x5               g155(.a(new_n250), .b(new_n246), .o1(\s[24] ));
  nanp02aa1n02x5               g156(.a(new_n245), .b(new_n244), .o1(new_n252));
  160nm_ficinv00aa1n08x5       g157(.clk(new_n252), .clkout(new_n253));
  nanb03aa1n02x5               g158(.a(new_n217), .b(new_n253), .c(new_n236), .out0(new_n254));
  nona32aa1n02x4               g159(.a(new_n222), .b(new_n252), .c(new_n229), .d(new_n227), .out0(new_n255));
  norp02aa1n02x5               g160(.a(\b[23] ), .b(\a[24] ), .o1(new_n256));
  aoi112aa1n02x5               g161(.a(\b[22] ), .b(\a[23] ), .c(\a[24] ), .d(\b[23] ), .o1(new_n257));
  aoi113aa1n02x5               g162(.a(new_n257), .b(new_n256), .c(new_n238), .d(new_n245), .e(new_n244), .o1(new_n258));
  nanp02aa1n02x5               g163(.a(new_n255), .b(new_n258), .o1(new_n259));
  160nm_ficinv00aa1n08x5       g164(.clk(new_n259), .clkout(new_n260));
  aoai13aa1n02x5               g165(.a(new_n260), .b(new_n254), .c(new_n181), .d(new_n187), .o1(new_n261));
  xorb03aa1n02x5               g166(.a(new_n261), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g167(.a(\b[24] ), .b(\a[25] ), .o1(new_n263));
  xorc02aa1n02x5               g168(.a(\a[25] ), .b(\b[24] ), .out0(new_n264));
  xorc02aa1n02x5               g169(.a(\a[26] ), .b(\b[25] ), .out0(new_n265));
  aoi112aa1n02x5               g170(.a(new_n263), .b(new_n265), .c(new_n261), .d(new_n264), .o1(new_n266));
  160nm_ficinv00aa1n08x5       g171(.clk(new_n263), .clkout(new_n267));
  nanp02aa1n02x5               g172(.a(new_n261), .b(new_n264), .o1(new_n268));
  160nm_ficinv00aa1n08x5       g173(.clk(new_n265), .clkout(new_n269));
  aoi012aa1n02x5               g174(.a(new_n269), .b(new_n268), .c(new_n267), .o1(new_n270));
  norp02aa1n02x5               g175(.a(new_n270), .b(new_n266), .o1(\s[26] ));
  nanp02aa1n02x5               g176(.a(new_n127), .b(new_n117), .o1(new_n272));
  nanp02aa1n02x5               g177(.a(new_n154), .b(new_n182), .o1(new_n273));
  nona22aa1n02x4               g178(.a(new_n273), .b(new_n186), .c(new_n183), .out0(new_n274));
  and002aa1n02x5               g179(.a(new_n265), .b(new_n264), .o(new_n275));
  nano22aa1n02x4               g180(.a(new_n237), .b(new_n275), .c(new_n253), .out0(new_n276));
  aoai13aa1n02x5               g181(.a(new_n276), .b(new_n274), .c(new_n272), .d(new_n180), .o1(new_n277));
  nanp02aa1n02x5               g182(.a(\b[25] ), .b(\a[26] ), .o1(new_n278));
  oai022aa1n02x5               g183(.a(\a[25] ), .b(\b[24] ), .c(\b[25] ), .d(\a[26] ), .o1(new_n279));
  aoi022aa1n02x5               g184(.a(new_n259), .b(new_n275), .c(new_n278), .d(new_n279), .o1(new_n280));
  xorc02aa1n02x5               g185(.a(\a[27] ), .b(\b[26] ), .out0(new_n281));
  xnbna2aa1n03x5               g186(.a(new_n281), .b(new_n277), .c(new_n280), .out0(\s[27] ));
  norp02aa1n02x5               g187(.a(\b[26] ), .b(\a[27] ), .o1(new_n283));
  160nm_ficinv00aa1n08x5       g188(.clk(new_n283), .clkout(new_n284));
  aobi12aa1n02x5               g189(.a(new_n281), .b(new_n277), .c(new_n280), .out0(new_n285));
  xnrc02aa1n02x5               g190(.a(\b[27] ), .b(\a[28] ), .out0(new_n286));
  nano22aa1n02x4               g191(.a(new_n285), .b(new_n284), .c(new_n286), .out0(new_n287));
  160nm_ficinv00aa1n08x5       g192(.clk(new_n275), .clkout(new_n288));
  nanp02aa1n02x5               g193(.a(new_n279), .b(new_n278), .o1(new_n289));
  aoai13aa1n02x5               g194(.a(new_n289), .b(new_n288), .c(new_n255), .d(new_n258), .o1(new_n290));
  aoai13aa1n02x5               g195(.a(new_n281), .b(new_n290), .c(new_n191), .d(new_n276), .o1(new_n291));
  aoi012aa1n02x5               g196(.a(new_n286), .b(new_n291), .c(new_n284), .o1(new_n292));
  norp02aa1n02x5               g197(.a(new_n292), .b(new_n287), .o1(\s[28] ));
  norb02aa1n02x5               g198(.a(new_n281), .b(new_n286), .out0(new_n294));
  aoai13aa1n02x5               g199(.a(new_n294), .b(new_n290), .c(new_n191), .d(new_n276), .o1(new_n295));
  oao003aa1n02x5               g200(.a(\a[28] ), .b(\b[27] ), .c(new_n284), .carry(new_n296));
  xnrc02aa1n02x5               g201(.a(\b[28] ), .b(\a[29] ), .out0(new_n297));
  aoi012aa1n02x5               g202(.a(new_n297), .b(new_n295), .c(new_n296), .o1(new_n298));
  aobi12aa1n02x5               g203(.a(new_n294), .b(new_n277), .c(new_n280), .out0(new_n299));
  nano22aa1n02x4               g204(.a(new_n299), .b(new_n296), .c(new_n297), .out0(new_n300));
  norp02aa1n02x5               g205(.a(new_n298), .b(new_n300), .o1(\s[29] ));
  xorb03aa1n02x5               g206(.a(new_n99), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g207(.a(new_n281), .b(new_n297), .c(new_n286), .out0(new_n303));
  aoai13aa1n02x5               g208(.a(new_n303), .b(new_n290), .c(new_n191), .d(new_n276), .o1(new_n304));
  oao003aa1n02x5               g209(.a(\a[29] ), .b(\b[28] ), .c(new_n296), .carry(new_n305));
  xnrc02aa1n02x5               g210(.a(\b[29] ), .b(\a[30] ), .out0(new_n306));
  aoi012aa1n02x5               g211(.a(new_n306), .b(new_n304), .c(new_n305), .o1(new_n307));
  aobi12aa1n02x5               g212(.a(new_n303), .b(new_n277), .c(new_n280), .out0(new_n308));
  nano22aa1n02x4               g213(.a(new_n308), .b(new_n305), .c(new_n306), .out0(new_n309));
  norp02aa1n02x5               g214(.a(new_n307), .b(new_n309), .o1(\s[30] ));
  norb02aa1n02x5               g215(.a(new_n303), .b(new_n306), .out0(new_n311));
  aobi12aa1n02x5               g216(.a(new_n311), .b(new_n277), .c(new_n280), .out0(new_n312));
  oao003aa1n02x5               g217(.a(\a[30] ), .b(\b[29] ), .c(new_n305), .carry(new_n313));
  xnrc02aa1n02x5               g218(.a(\b[30] ), .b(\a[31] ), .out0(new_n314));
  nano22aa1n02x4               g219(.a(new_n312), .b(new_n313), .c(new_n314), .out0(new_n315));
  aoai13aa1n02x5               g220(.a(new_n311), .b(new_n290), .c(new_n191), .d(new_n276), .o1(new_n316));
  aoi012aa1n02x5               g221(.a(new_n314), .b(new_n316), .c(new_n313), .o1(new_n317));
  norp02aa1n02x5               g222(.a(new_n317), .b(new_n315), .o1(\s[31] ));
  160nm_ficinv00aa1n08x5       g223(.clk(new_n104), .clkout(new_n319));
  xnbna2aa1n03x5               g224(.a(new_n101), .b(new_n105), .c(new_n319), .out0(\s[3] ));
  oaoi03aa1n02x5               g225(.a(\a[3] ), .b(\b[2] ), .c(new_n101), .o1(new_n321));
  xorb03aa1n02x5               g226(.a(new_n321), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g227(.a(new_n108), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  oao003aa1n02x5               g228(.a(new_n122), .b(new_n123), .c(new_n108), .carry(new_n324));
  xorb03aa1n02x5               g229(.a(new_n324), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  norb02aa1n02x5               g230(.a(new_n112), .b(new_n111), .out0(new_n326));
  160nm_ficinv00aa1n08x5       g231(.clk(new_n121), .clkout(new_n327));
  aoai13aa1n02x5               g232(.a(new_n326), .b(new_n124), .c(new_n324), .d(new_n327), .o1(new_n328));
  aoi112aa1n02x5               g233(.a(new_n326), .b(new_n124), .c(new_n324), .d(new_n327), .o1(new_n329));
  norb02aa1n02x5               g234(.a(new_n328), .b(new_n329), .out0(\s[7] ));
  norb02aa1n02x5               g235(.a(new_n110), .b(new_n109), .out0(new_n331));
  160nm_ficinv00aa1n08x5       g236(.clk(new_n111), .clkout(new_n332));
  xnbna2aa1n03x5               g237(.a(new_n331), .b(new_n328), .c(new_n332), .out0(\s[8] ));
  xobna2aa1n03x5               g238(.a(new_n128), .b(new_n127), .c(new_n117), .out0(\s[9] ));
endmodule


